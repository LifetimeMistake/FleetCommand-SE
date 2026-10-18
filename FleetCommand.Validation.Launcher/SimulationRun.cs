using System;
using System.IO;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation.Launcher
{
    public static class SimulationRun
    {
        public static string DefaultBlueprint => Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
            "blueprints", "FleetCommand_Validation_v1", "bp.sbc");

        public static string Execute(string blueprint, Vector3D position, MatrixD orientation,
            Action<FleetTestContext, SimShip, ValidationRunner> afterStart = null,
            Action<FleetTestContext, SimShip> beforeStart = null)
        {
            PhysicsHost.Initialize();
            var file = new SbcFileReader().ReadFile(blueprint);
            var template = SbcToShipTemplate.Convert(file, file.Name);
            using (var context = new FleetTestContext(new SimWorld(SePhysicsProfile.Default, position)))
            {
                var ship = context.Spawn(template, SpawnState.At(position, orientation));
                context.Step(2); // Publish initial working/power state before preflight.
                var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                var lcd = (IMyTextPanel)ship.TerminalSystem.GetBlockWithName("FC Report");
                var runner = new ValidationRunner(ship.TerminalSystem, pb.Runtime, pb, _ => { }, pb.Storage);
                beforeStart?.Invoke(context, ship);
                UpdateType source = UpdateType.Terminal;
                pb.RunHandler = arg => {
                    runner.Main(arg, source);
                    if (!runner.IsRunning) pb.Storage = runner.Report;
                };
                pb.TryRun("start");
                // Permit a scheduled callback in the same frame as terminal start,
                // as the real game does. The shared runner handles the zero interval.
                afterStart?.Invoke(context, ship, runner);
                context.EveryUpdate1(frame => {
                    if ((pb.Runtime.UpdateFrequency & UpdateFrequency.Update1) == 0) return;
                    source = frame.UpdateSource;
                    pb.TryRun("");
                });
                int ticks = 0;
                while (runner.IsRunning && ticks++ < 25000) context.Step();
                if (runner.IsRunning) { source = UpdateType.Terminal; pb.TryRun("abort"); }
                context.Step(10); // Let UpdateAfterSimulation10 publish the final LCD buffer.
                // Read only the external LCD API: no hidden runner telemetry escapes here.
                string report = lcd.GetText();
                if (report != runner.Report) throw new InvalidOperationException("LCD does not contain the complete canonical report.");
                return report;
            }
        }
    }
}
