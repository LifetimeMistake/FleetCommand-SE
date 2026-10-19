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
        public static string StationBlueprint => Path.Combine(AppDomain.CurrentDomain.BaseDirectory,
            "blueprints", "FleetCommand_Validation_Station_v1", "bp.sbc");

        public static string Execute(string blueprint, Vector3D position, MatrixD orientation,
            Action<FleetTestContext, SimShip, IValidationExperiment> afterStart = null,
            Action<FleetTestContext, SimShip> beforeStart = null, string command = "start",
            MatrixD? referenceOrientation = null, Action<string, string> additionalArtifact = null)
        {
            PhysicsHost.Initialize();
            var file = new SbcFileReader().ReadFile(blueprint);
            using (var context = new FleetTestContext(new SimWorld(SePhysicsProfile.Default, position)))
            {
                var ship = AssemblySpawn.Spawn(context, file, position, orientation, referenceOrientation);
                context.Step(2); // Publish initial working/power state before preflight.
                var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                var lcd = (IMyTextPanel)ship.TerminalSystem.GetBlockWithName(command == "dockprobe" ? "FC Dock Report" : "FC Report");
                var runner = new ValidationHost(ship.TerminalSystem, pb.Runtime, pb, _ => { }, pb.Storage);
                beforeStart?.Invoke(context, ship);
                pb.RunHandler = arg => {
                    runner.Main(arg, pb.CurrentUpdateSource);
                    if (!runner.IsRunning) pb.Storage = runner.Report;
                };
                pb.Run(command, UpdateType.Terminal);
                afterStart?.Invoke(context, ship, runner);
                int ticks = 0;
                while (runner.IsRunning && ticks++ < 40000) context.Step();
                if (runner.IsRunning) pb.Run("abort", UpdateType.Terminal);
                context.Step(10); // Let UpdateAfterSimulation10 publish the final LCD buffer.
                // Read only the external LCD API: no hidden runner telemetry escapes here.
                string report = lcd.GetText();
                if (report != runner.Report) throw new InvalidOperationException("LCD does not contain the complete canonical report.");
                if (command == "dockprobe" && additionalArtifact != null)
                    additionalArtifact("flight", ((IMyTextPanel)ship.TerminalSystem.GetBlockWithName("FC Report")).GetText());
                return report;
            }
        }
    }
}
