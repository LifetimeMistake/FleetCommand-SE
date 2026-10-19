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
        [TestCase(0, 0, 0, 1.86446, 0.009983)]
        [TestCase(0.7, -0.3, 1.2, 0.011378, 0.006405)]
        public void LowSpeedGyroInteraction_PreservesNativeWorldOrientationDependence(
            double yaw, double pitch, double roll, double finalVelocity, double finalRate)
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero,
                MatrixD.CreateFromYawPitchRoll(yaw, pitch, roll), command: "restprobe");
            var sample = report.Split('\n').Single(l => l.StartsWith("S|low_gyro_spin|300|")).Split('|');
            Assert.That(double.Parse(sample[8], System.Globalization.CultureInfo.InvariantCulture),
                Is.EqualTo(finalVelocity).Within(0.00003));
            Assert.That(double.Parse(sample[11], System.Globalization.CultureInfo.InvariantCulture),
                Is.EqualTo(finalRate).Within(0.000003));
        }

        [Test]
        public void InitialOrientation_IsExportedAndCanReplayTheReferenceFrame()
        {
            MatrixD orientation = MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2);
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero,
                orientation, command: "restprobe");
            foreach (string component in new[] { "qx", "qy", "qz", "qw" })
                Assert.That(report, Does.Contain("META|initial_" + component + "|"));
            MatrixD recorded = ReportInitialOrientation.Read(report);
            Assert.That(Vector3D.Distance(recorded.Right, orientation.Right), Is.LessThan(0.000003));
            Assert.That(Vector3D.Distance(recorded.Up, orientation.Up), Is.LessThan(0.000003));
            string replay = SimulationRun.Execute(SimulationRun.DefaultBlueprint, new Vector3D(100, 200, 300),
                MatrixD.Identity, command: "restprobe", referenceOrientation: recorded);
            var originalFinal = report.Split('\n').Single(l => l.StartsWith("S|low_gyro_spin|300|")).Split('|');
            var replayFinal = replay.Split('\n').Single(l => l.StartsWith("S|low_gyro_spin|300|")).Split('|');
            Assert.That(double.Parse(replayFinal[8], System.Globalization.CultureInfo.InvariantCulture),
                Is.EqualTo(double.Parse(originalFinal[8], System.Globalization.CultureInfo.InvariantCulture)).Within(0.00001));
        }

        [TestCase("")]
        [TestCase("META|initial_qx|NaN\nMETA|initial_qy|0\nMETA|initial_qz|0\nMETA|initial_qw|1")]
        [TestCase("META|initial_qx|0\nMETA|initial_qy|0\nMETA|initial_qz|0\nMETA|initial_qw|0")]
        [TestCase("META|initial_qx|0\nMETA|initial_qx|0\nMETA|initial_qy|0\nMETA|initial_qz|0\nMETA|initial_qw|1")]
        public void InitialOrientation_RejectsMissingOrMalformedTelemetry(string report)
        {
            Assert.Throws<FormatException>(new Action(() => { ReportInitialOrientation.Read(report); }));
        }

        [Test]
        public void LowSpeedDiagnostic_ConnectorDetectorsReproduceStallAndTheirMovingMass()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "restprobe");
            var lines = report.Split('\n');
            var held = lines.Single(l => l.StartsWith("S|low_hold|300|")).Split('|');
            Assert.That(double.Parse(held[8], System.Globalization.CultureInfo.InvariantCulture), Is.Zero);
            var movingStart = lines.Single(l => l.StartsWith("S|low_moving|0|")).Split('|');
            var movingEnd = lines.Single(l => l.StartsWith("S|low_moving|300|")).Split('|');
            double mass = double.Parse(lines.Single(l => l.StartsWith("META|mass_kg|")).Split('|')[2], System.Globalization.CultureInfo.InvariantCulture);
            double gain = double.Parse(movingEnd[8], System.Globalization.CultureInfo.InvariantCulture)
                - double.Parse(movingStart[8], System.Globalization.CultureInfo.InvariantCulture);
            Assert.That(gain, Is.EqualTo(2880 * 5 / (mass + 2)).Within(0.0001),
                "The source creates two native default-mass (1kg) detector bodies not included in CalculateShipMass.");
        }

        [TestCase(0, 0, 0)]
        [TestCase(0.6, -0.3, 0.4)]
        public void LowSpeedDiagnostic_LeavesReportHeadroomWithArbitraryPlacement(double yaw, double pitch, double roll)
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint,
                new Vector3D(2300, -4100, 700), MatrixD.CreateFromYawPitchRoll(yaw, pitch, roll), command: "restprobe");
            Assert.That(report, Does.Contain("END|COMPLETE|"));
            Assert.That(report.Length, Is.LessThan(35000), "Leave room for longer game numeric fields and warmup telemetry.");
            // Stress continuous fields with signed six-digit integer parts and six
            // decimals, plus the maximum warmup row count. Ticks and flags stay integers.
            var padded = report.Split('\n').Select(line => {
                var fields = line.Split('|');
                if (fields[0] == "S" || fields[0] == "U" || fields[0] == "GY")
                    for (int i = 3; i < fields.Length; i++)
                    {
                        bool flag = fields[0] == "S" && i >= 28 && i <= 30
                            || fields[0] == "U" && i == 9
                            || fields[0] == "GY" && i >= 3 && i <= 5;
                        if (!flag) fields[i] = "-999999.999999";
                    }
                return string.Join("|", fields);
            }).ToArray();
            var warmup = padded.Where(line => line.StartsWith("U|")).ToArray();
            int stressedLength = string.Join("\n", padded).Length + (88 - warmup.Length) * (warmup[0].Length + 1);
            Assert.That(stressedLength, Is.LessThan(ValidationReport.Limit - 2000));
        }

        [TestCase(false)]
        [TestCase(true)]
        public void LowSpeedGyroControl_ChangesNativeDeactivationWithoutTurningThrust(bool spinning)
        {
            bool observed = false;
            string caseName = spinning ? "low_gyro_spin" : "low_gyro_zero";
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint,
                new Vector3D(2300, -4100, 700), MatrixD.CreateFromYawPitchRoll(0.6, -0.3, 0.4),
                afterStart: (context, ship, runner) => context.EveryUpdate1(_ => {
                    if (observed || !runner.Report.Contains("S|" + caseName + "|10|")) return;
                    var gyro = (IMyGyro)ship.TerminalSystem.GetBlockWithName("FC Gyro A");
                    var thrust = (IMyThrust)ship.TerminalSystem.GetBlockWithName("FC Thrust +X");
                    var reference = (IMyRemoteControl)ship.TerminalSystem.GetBlockWithName("FC Reference");
                    var native = (Havok.HkRigidBody)typeof(SimRigidBody).GetProperty("Body",
                        System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).GetValue(ship.Grid.Body);
                    Assert.That(gyro.Enabled && gyro.IsWorking && gyro.GyroOverride, Is.True);
                    Assert.That(native.EnableDeactivation, Is.EqualTo(!spinning));
                    var target = Vector3D.TransformNormal(new Vector3D(-gyro.Pitch, -gyro.Yaw, -gyro.Roll), gyro.WorldMatrix);
                    Assert.That(target.Length(), Is.EqualTo(spinning ? 0.01 : 0).Within(1e-6));
                    Assert.That(Vector3D.Cross(target, thrust.WorldMatrix.Backward).Length(), Is.LessThan(1e-6));
                    Assert.That(thrust.ThrustOverridePercentage, Is.EqualTo(0.2f));
                    Assert.That(thrust.CurrentThrust, Is.EqualTo(2880).Within(0.01));
                    Assert.That(reference.DampenersOverride, Is.False);
                    observed = true;
                }), command: "restprobe");
            Assert.That(observed, Is.True, "The native deactivation control must actually execute.");
            Assert.That(report, Does.Contain("ENDCASE|" + caseName + "|18"));
        }

        [Test]
        public void LowMoving_BeginsMeasurementOnlyAfterMeasuredSpeedReachesOneMetrePerSecond()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "restprobe");
            var initial = report.Split('\n').Single(l => l.StartsWith("S|low_moving|0|")).Split('|');
            Assert.That(double.Parse(initial[8], System.Globalization.CultureInfo.InvariantCulture), Is.GreaterThanOrEqualTo(1));
            Assert.That(report, Does.Contain("WARMUPEND|low_moving|"));
        }

        [Test]
        public void LowMoving_FailsExplicitlyIfWarmupCannotReachMeasuredSpeed()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                afterStart: (context, ship, runner) => context.EveryUpdate1(_ => {
                    if (!runner.Report.Contains("WARMUP|low_moving|") || runner.Report.Contains("WARMUPEND|low_moving|")) return;
                    var state = ship.Grid.Body.GetState();
                    state.LinearVelocity = Vector3D.Zero;
                    state.AngularVelocity = Vector3.Zero;
                    ship.Grid.Body.SetState(state);
                }), command: "restprobe");
            Assert.That(report, Does.Contain("END|ERROR|WARMUP_TIMEOUT_LOW_MOVING|"));
            Assert.That(report, Does.Not.Contain("CASE|low_moving|"));
        }

        [Test]
        public void LowSpeedDiagnostic_AcceptsFullBatteryWithoutChangingCanonicalPreflight()
        {
            Action<FleetCommand.Test.Framework.FleetTestContext, SimShip> fill = (context, ship) => {
                var battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("FC Battery");
                battery.CurrentStoredPower = battery.MaxStoredPower;
            };
            string diagnostic = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: fill, command: "restprobe");
            Assert.That(diagnostic, Does.Contain("END|COMPLETE|6|108"));
            string canonical = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                beforeStart: fill);
            Assert.That(canonical, Does.Contain("END|ERROR|BATTERY_MUST_BE_0.02_TO_0.04_MWH|"));
        }

        [Test]
        public void LowSpeedDiagnostic_ExportsAllSixCasesAndTransitionSamplesUnderLcdCap()
        {
            string report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity,
                command: "restprobe");
            Assert.That(report, Does.StartWith("FCDIAGNOSTIC|1|craft-v1|low-speed-v5\n"));
            Assert.That(report, Does.Contain("END|COMPLETE|6|108"));
            Assert.That(report.Length, Is.LessThan(64000));
            var lines = report.Split('\n');
            foreach (var name in new[] {"low_hold", "low_wake", "low_refresh", "low_moving", "low_gyro_zero", "low_gyro_spin"})
            {
                Assert.That(lines.Any(l => l.StartsWith("S|" + name + "|1|")), Is.True);
                foreach (var row in lines.Where(l => l.StartsWith("S|" + name + "|")))
                    Assert.That(row.Split('|')[28], Is.EqualTo("0"), "Every measured interval has dampeners off.");
            }
            Assert.That(lines.Count(l => l.StartsWith("GY|")), Is.EqualTo(6));
            foreach (var name in new[] { "low_gyro_zero", "low_gyro_spin" })
                foreach (int tick in new[] { 0, 10, 300 })
                    Assert.That(lines.Any(l => l.StartsWith("GY|" + name + "|" + tick + "|")), Is.True);
            for (int tick = 1; tick <= 12; tick++)
                Assert.That(lines.Any(l => l.StartsWith("S|low_moving|" + tick + "|")), Is.True);
        }

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
                    Assert.That(thrusters[0].CurrentThrust, Is.EqualTo(14400).Within(0.1),
                        "The first thrust update precedes distribution of the new load.");
                    context.Step();
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
            Assert.That(report.Length, Is.LessThan(65000), "Complete report must survive the game's LCD publication cap.");
            Assert.That(report.Split('\n').Count(l => l.StartsWith("ENDCASE|")), Is.EqualTo(19));
            Assert.That(report, Does.Contain("COVERAGE|manual|"));
            Assert.That(report, Does.Contain("COVERAGE|uncovered|"));
            var waypoint = report.Split('\n').Where(l => l.StartsWith("W|")).ToArray();
            Assert.That(waypoint.Length, Is.EqualTo(88), "Initial state, every frame in the first second, then 20-frame checkpoints.");
            for (int tick = 0; tick <= 60; tick++)
                Assert.That(waypoint.Any(l => l.StartsWith("W|" + tick + "|")), Is.True);
            Assert.That(waypoint.Last(), Does.StartWith("W|600|"));
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
