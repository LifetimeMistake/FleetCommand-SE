using System;
using System.Linq;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Validation.Launcher;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

[assembly: NonParallelizable]
[assembly: LevelOfParallelism(1)]

namespace FleetCommand.Validation.Tests
{
    [SetUpFixture]
    public class PhysicsSetup
    {
        [OneTimeSetUp] public void Start() { PhysicsHost.Initialize(); }
        [OneTimeTearDown] public void Stop() { PhysicsHost.Shutdown(); }
    }

    public class ValidationTests
    {
        [Test]
        public void FullRun_LogsPassiveGateAndFrameTransitionsWithinOneLcd()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity);
            var lines = report.Split('\n');
            Assert.That(report, Does.Contain("END|COMPLETE|19|133"));
            Assert.That(lines.Count(l => l.StartsWith("GATE|")), Is.EqualTo(19));
            Assert.That(lines.Where(l => l.StartsWith("SETTLE|")).All(l => int.Parse(l.Split('|')[2]) >= 75), Is.True);
            Assert.That(report, Does.Contain("DCOLUMNS|case|phase|run_tick|phase_tick|dt|"));
            foreach (var name in new[] { "thrust_px", "coast_px", "thrust_py" })
            {
                Assert.That(report, Does.Contain("D|" + name + "|passive|"));
                Assert.That(report, Does.Contain("D|" + name + "|command|"));
                for (int tick = 1; tick <= 12; tick++)
                    Assert.That(lines.Any(l => l.StartsWith("D|" + name + "|flight|") && l.Split('|')[4] == tick.ToString()), Is.True);
            }
            var footer = lines.Single(l => l.StartsWith("TRACE|rows|")).Split('|');
            Assert.That(footer[4], Is.EqualTo("0"), "Diagnostic rows must not be dropped in the normal suite.");
            Assert.That(report.Length, Is.LessThan(ValidationReport.Limit));
            var headers = ValidationTrace.Columns.Split('|');
            int gyroMask = Array.IndexOf(headers, "gyro_enabled_mask");
            int gyroOverride = Array.IndexOf(headers, "gyro_override_mask");
            int thrustOverride = Array.IndexOf(headers, "override_px");
            foreach (var row in lines.Where(l => l.StartsWith("D|") && l.Split('|')[2] == "passive"))
            {
                var fields = row.Split('|');
                Assert.That(double.Parse(fields[gyroMask], System.Globalization.CultureInfo.InvariantCulture), Is.Zero);
                Assert.That(double.Parse(fields[gyroOverride], System.Globalization.CultureInfo.InvariantCulture), Is.Zero);
                for (int i = 0; i < 6; i++)
                    Assert.That(double.Parse(fields[thrustOverride + i], System.Globalization.CultureInfo.InvariantCulture), Is.Zero);
            }
            var release = lines.Single(l => l.StartsWith("D|coast_px|command|") && l.Split('|')[4] == "15").Split('|');
            Assert.That(double.Parse(release[thrustOverride], System.Globalization.CultureInfo.InvariantCulture), Is.Zero);
            for (int tick = 16; tick <= 27; tick++)
                Assert.That(lines.Any(l => l.StartsWith("D|coast_px|flight|") && l.Split('|')[4] == tick.ToString()), Is.True);
        }

        [Test]
        public void ImportedBattery_ReenableDoesNotChargeForDisabledFrames()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    var battery = (IMyBatteryBlock)ship.TerminalSystem.GetBlockWithName("FC Battery");
                    battery.Enabled = false;
                    context.Step(248); // completed frame250
                    float initial = battery.CurrentStoredPower;
                    battery.Enabled = true;
                    battery.ChargeMode = ChargeMode.Recharge;
                    context.Step(50); // next energy update at300 includes only50 enabled frames
                    Assert.That(battery.CurrentStoredPower - initial,
                        Is.EqualTo(0.2 * (50.0 / 60) / 3600 * 0.8).Within(1e-8));
                    battery.ChargeMode = ChargeMode.Auto;
                });
        }

        [Test]
        public void ImportedPowerGroups_ServeGyrosBeforeAdaptableThrusters()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    var reactors = new System.Collections.Generic.List<IMyReactor>();
                    var thrusters = new System.Collections.Generic.List<IMyThrust>();
                    var gyros = new System.Collections.Generic.List<IMyGyro>();
                    ship.TerminalSystem.GetBlocksOfType(reactors);
                    ship.TerminalSystem.GetBlocksOfType(thrusters);
                    ship.TerminalSystem.GetBlocksOfType(gyros);
                    foreach (var r in reactors) r.Enabled = false;
                    foreach (var t in thrusters) t.ThrustOverridePercentage = 1f;
                    var battery = (IMyBatteryBlock)ship.TerminalSystem.GetBlockWithName("FC Battery");
                    battery.ChargeMode = ChargeMode.Discharge;
                    context.Step();
                    Assert.That(gyros.All(g => g.IsWorking), Is.True);
                    Assert.That(((SimGyro)gyros[0]).DeliveredRatio, Is.EqualTo(1f));
                    Assert.That(thrusters[0].CurrentThrust, Is.EqualTo(14400 * ((0.2 - 0.0075401) / 1.2)).Within(0.1));
                    foreach (var t in thrusters) t.ThrustOverridePercentage = 0;
                    foreach (var r in reactors) r.Enabled = true;
                    battery.ChargeMode = ChargeMode.Auto;
                    context.Step();
                });
        }

        [Test]
        public void ImportedBattery_UpdatesStoredEnergyEvery100Frames_WithDefinitionEfficiency()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    var battery = (IMyBatteryBlock)ship.TerminalSystem.GetBlockWithName("FC Battery");
                    battery.ChargeMode = ChargeMode.Recharge;
                    float initial = battery.CurrentStoredPower;
                    context.Step(97); // spawned world has already completed two ticks
                    Assert.That(battery.CurrentStoredPower, Is.EqualTo(initial));
                    context.Step();
                    Assert.That(battery.CurrentStoredPower - initial,
                        Is.EqualTo(0.2 * (100.0 / 60) / 3600 * 0.8).Within(1e-8));
                    battery.ChargeMode = ChargeMode.Auto;
                });
        }

        [Test]
        public void FixtureMass_PreservesFractionalComponentMasses()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => Assert.That(ship.Grid.Body.Mass, Is.EqualTo(7551.8).Within(0.01)));
        }

        [Test]
        public void FixtureIdlePower_ChargesFromMeasuredProducerOutput()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    context.Step(10);
                    var battery = (IMyBatteryBlock)ship.TerminalSystem.GetBlockWithName("FC Battery");
                    var reactors = new System.Collections.Generic.List<IMyReactor>();
                    ship.TerminalSystem.GetBlocksOfType(reactors);
                    Assert.That(battery.CurrentOutput, Is.EqualTo(0.007552).Within(0.00002));
                    Assert.That(battery.CurrentInput, Is.EqualTo(0.2).Within(0.00001));
                    Assert.That(reactors.Sum(r => r.CurrentOutput), Is.EqualTo(0.2).Within(0.00002));
                });
        }

        [Test]
        public void LcdReadback_IsPublishedOnSimulationUpdateRatherThanSynchronously()
        {
            SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    var lcd = (IMyTextPanel)ship.TerminalSystem.GetBlockWithName("FC Report");
                    string previous = lcd.GetText();
                    lcd.WriteText("deferred");
                    Assert.That(lcd.GetText(), Is.EqualTo(previous));
                    context.Step(10);
                    Assert.That(lcd.GetText(), Is.EqualTo("deferred"));
                });
        }

        [Test]
        public void FixtureBatteryMode_UsesTheGameIntegerXmlField()
        {
            var doc = System.Xml.Linq.XDocument.Load(SimulationRun.DefaultBlueprint);
            foreach (var mode in doc.Descendants("ChargeMode"))
                Assert.That(int.TryParse(mode.Value, out _), Is.True, "MyObjectBuilder_BatteryBlock.ChargeMode is an int, not an XML enum.");
        }

        [Test]
        public void FullRun_ExportsCompleteBoundedLcdReport()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity);
            TestContext.AddTestAttachment(WriteArtifact("validation-report.txt", report));
            Assert.That(report, Does.Contain("END|COMPLETE|19|"), report.Substring(Math.Max(0, report.Length - 1000)));
            Assert.That(report.Length, Is.LessThan(ValidationReport.Limit));
            Assert.That(report.Split('\n').Count(l => l.StartsWith("ENDCASE|")), Is.EqualTo(19));
            Assert.That(report, Does.Contain("COVERAGE|manual|"));
            Assert.That(report, Does.Contain("COVERAGE|uncovered|"));
        }

        [Test]
        public void Abort_ClearsActuatorsAndLeavesExtractableLcdReport()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                (context, ship, runner) => {
                    context.AtTick(context.Tick + 100, _ => {
                        runner.Main("abort", UpdateType.Terminal);
                        var thrusters = new System.Collections.Generic.List<IMyThrust>();
                        ship.TerminalSystem.GetBlocksOfType(thrusters);
                        Assert.That(thrusters.All(t => t.ThrustOverridePercentage == 0), Is.True);
                        var gyros = new System.Collections.Generic.List<IMyGyro>();
                        ship.TerminalSystem.GetBlocksOfType(gyros);
                        Assert.That(gyros.All(g => !g.GyroOverride), Is.True);
                        Assert.That(((IMyRemoteControl)ship.Controller).IsAutoPilotEnabled, Is.False);
                    });
                });
            Assert.That(report, Does.Contain("END|ABORTED|OPERATOR"));
        }

        [Test]
        public void SameFrameFirstUpdate_DoesNotFailCadenceOrAdvanceTheSuite()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                afterStart: (context, ship, runner) => {
                    var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                    // Model a scheduled invocation in the same frame as a terminal run.
                    pb.Runtime.MarkRun();
                    runner.Main("", UpdateType.Update1);
                    Assert.That(runner.IsRunning, Is.True, runner.Report);
                    Assert.That(runner.Report, Does.Not.Contain("CASE|"));
                });
            Assert.That(report, Does.Contain("END|COMPLETE|19|133"));
            Assert.That(report, Does.Contain("INFO|startup_zero_elapsed_callbacks|"));
        }

        [Test]
        public void ZeroIntervalDuringMeasurement_StillFailsCadence()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                afterStart: (context, ship, runner) => context.AtTick(context.Tick + 100, _ => {
                    var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                    pb.Runtime.MarkRun();
                    runner.Main("", UpdateType.Update1);
                }));
            Assert.That(report, Does.Contain("END|ERROR|CADENCE_0.000000"));
        }

        [Test]
        public void MissedCallback_IsReportedInsteadOfSilentlyComparingDifferentTiming()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                (context, ship, runner) => context.AtTick(context.Tick + 50, _ => context.World.SuppressNextUpdate1()));
            Assert.That(report, Does.Contain("END|ERROR|CADENCE_"));
        }

        [Test]
        public void GravityPreflight_RejectsUnsuitableWorld()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => context.SetGravity(SimGravityField.Uniform(new Vector3(0, -9.81f, 0))));
            Assert.That(report, Does.Contain("END|ERROR|VACUUM_ZERO_GRAVITY_REQUIRED"));
        }

        [Test]
        public void MovingPreflight_RejectsUnknownInitialMotion()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    var state = SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(1, 0, 0)).ToBodyState(ship.Grid.EntityId);
                    ship.Grid.Body.SetState(in state);
                });
            Assert.That(report, Does.Contain("END|ERROR|START_AT_REST"));
        }

        [Test]
        public void MissingActuator_ProducesPreflightErrorOnLcd()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: (context, ship) => {
                    ship.TerminalSystem.GetBlockWithName("FC Thrust +X").CustomName = "misnamed";
                    // Reconstruct after the rename, as an ingame recompile would.
                    var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                    var runner = new ValidationRunner(ship.TerminalSystem, pb.Runtime, pb, _ => { }, "");
                    runner.Main("start", UpdateType.Terminal);
                    Assert.That(runner.Report, Does.Contain("END|ERROR|MISSING_THRUSTER"));
                    ship.TerminalSystem.GetBlockWithName("misnamed").CustomName = "FC Thrust +X";
                });
            Assert.That(report, Does.Contain("END|COMPLETE|"));
        }

        [Test]
        public void Reload_PreservesCompletedReportOnLcd()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                (context, ship, runner) => {
                    runner.Main("abort", UpdateType.Terminal);
                    var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                    var restored = new ValidationRunner(ship.TerminalSystem, pb.Runtime, pb, _ => { }, runner.Report);
                    Assert.That(restored.IsRunning, Is.False);
                    Assert.That(restored.Report, Is.EqualTo(runner.Report));
                });
            Assert.That(report, Does.Contain("END|ABORTED|"));
        }

        static string WriteArtifact(string name, string text)
        {
            string path = System.IO.Path.Combine(TestContext.CurrentContext.WorkDirectory, name);
            System.IO.File.WriteAllText(path, text);
            return path;
        }
    }
}
