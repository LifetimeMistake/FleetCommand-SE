using System;
using System.Globalization;
using System.Linq;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Sbc;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using FleetCommand.Validation.Launcher;
using NUnit.Framework;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace FleetCommand.Validation.Tests
{
    public class FlightProbeTests
    {
        static double N(string[] row, int index) { return double.Parse(row[index], CultureInfo.InvariantCulture); }

        [Test]
        public void FlightProbe_NewThrustLoadUsesExistingSupplyRatioUntilNextDistribution()
        {
            string report = Execute();
            var columns = report.Split('\n').Single(l => l.StartsWith("COLUMNS|")).Split('|');
            int thrust = Array.IndexOf(columns, "thrust_px");
            foreach (string name in new[] { "power_limited_rest", "power_limited_moving" })
            {
                var rows = report.Split('\n').Where(l => l.StartsWith("S|" + name + "|")).Select(l => l.Split('|')).ToArray();
                Assert.That(N(rows.Single(r => r[2] == "1"), thrust), Is.EqualTo(14400).Within(0.01),
                    "The new five-thruster load is published by the thrust update after distribution; its first frame retains the supplied ratio from the preceding low load.");
                Assert.That(N(rows.Single(r => r[2] == "2"), thrust), Is.LessThan(14400 * 0.3),
                    "The following distribution must apply the battery shortage.");
            }
        }

        [Test]
        public void FlightProbe_CollectsFiniteUncancelledPowerAndSteeringThroughBraking()
        {
            string report = Execute();
            Assert.That(report, Does.StartWith("FCDIAGNOSTIC|1|craft-v1|flight-v1\n"));
            Assert.That(report, Does.Contain("END|COMPLETE|6|"));
            Assert.That(report.Length, Is.LessThan(ValidationReport.Limit));
            var columns = report.Split('\n').Single(l => l.StartsWith("COLUMNS|")).Split('|');
            Func<string, int> index = name => Array.IndexOf(columns, name);
            var samples = report.Split('\n').Where(l => l.StartsWith("S|")).Select(l => l.Split('|')).ToArray();
            foreach (var row in samples)
            {
                Assert.That(row.Length, Is.EqualTo(columns.Length));
                for (int i = 2; i < row.Length; i++) Assert.That(double.IsNaN(N(row, i)) || double.IsInfinity(N(row, i)), Is.False);
            }
            foreach (string name in new[] { "power_full_rest", "power_limited_rest", "power_full_moving", "power_limited_moving" })
            {
                var rows = samples.Where(r => r[1] == name).ToArray();
                Assert.That(rows.Length, Is.GreaterThan(10));
                foreach (var row in rows)
                {
                    Assert.That(N(row, index("thrust_nx")), Is.Zero, "The net +X measurement must not be canceled by -X thrust.");
                    Assert.That(N(row, index("thrust_py")), Is.EqualTo(N(row, index("thrust_ny"))).Within(0.001));
                    Assert.That(N(row, index("thrust_pz")), Is.EqualTo(N(row, index("thrust_nz"))).Within(0.001));
                }
                Assert.That(rows.Skip(1).Max(r => N(r, index("thrust_py"))), Is.GreaterThan(0), "Orthogonal paired loads create a measurable power shortage.");
                Assert.That(rows.Skip(1).Max(r => N(r, index("thrust_px"))), Is.GreaterThan(0));
                if (name.EndsWith("moving"))
                {
                    Assert.That(N(rows[0], index("speed")), Is.GreaterThanOrEqualTo(1));
                    Assert.That(N(rows.Last(), index("speed")), Is.GreaterThan(N(rows[0], index("speed"))));
                    Assert.That(Vector3D.Distance(new Vector3D(N(rows[0], index("px")), N(rows[0], index("py")), N(rows[0], index("pz"))),
                        new Vector3D(N(rows.Last(), index("px")), N(rows.Last(), index("py")), N(rows.Last(), index("pz")))), Is.GreaterThan(1));
                }
            }
            var full = samples.Where(r => r[1] == "power_full_moving" && N(r, 2) >= 60).ToArray();
            var limited = samples.Where(r => r[1] == "power_limited_moving" && N(r, 2) >= 60).ToArray();
            Assert.That(limited.Max(r => N(r, index("thrust_px"))), Is.LessThan(full.Min(r => N(r, index("thrust_px"))) * 0.3));
            var fullMoving = samples.Where(r => r[1] == "power_full_moving").ToArray();
            var limitedMoving = samples.Where(r => r[1] == "power_limited_moving").ToArray();
            double fullGain = N(fullMoving.Last(), index("speed")) - N(fullMoving.First(), index("speed"));
            double limitedGain = N(limitedMoving.Last(), index("speed")) - N(limitedMoving.First(), index("speed"));
            Assert.That(limitedGain, Is.LessThan(fullGain * 0.3), "Power starvation must change measured acceleration, not only a telemetry flag.");
            Assert.That(limited.All(r => N(r, index("reactor_mw")) == 0), Is.True);
            Assert.That(limited.All(r => N(r, index("battery_out")) > 0), Is.True);
            foreach (string name in new[] { "waypoint_forward", "waypoint_turn" })
            {
                var rows = samples.Where(r => r[1] == name).ToArray();
                Assert.That(rows.Count(r => N(r, 2) <= 30), Is.EqualTo(31));
                Assert.That(N(rows[0], index("distance")), Is.EqualTo(30).Within(0.001));
                Assert.That(rows.Last()[2], Is.EqualTo("1500"));
                Assert.That(rows.Skip(1).Min(r => N(r, index("distance"))), Is.LessThan(28));
                Assert.That(rows.Max(r => N(r, index("speed"))), Is.GreaterThan(1));
                Assert.That(N(rows.Last(), index("speed")), Is.LessThan(rows.Max(r => N(r, index("speed")))));
            }
            var turning = samples.Where(r => r[1] == "waypoint_turn").ToArray();
            Assert.That(turning.Max(r => Math.Abs(N(r, index("wy")))), Is.GreaterThan(0.01));
            Assert.That(report, Does.Contain("META|initial_qw|"));
            Assert.That(report.Split('\n').Count(l => l.StartsWith("GATE|")), Is.EqualTo(6));
            TestContext.Out.WriteLine(report);
        }

        [Test]
        public void FlightProbe_AbortCleansCommandsAndPublishesCompleteFooter()
        {
            string report = Execute(abort: true);
            Assert.That(report, Does.Contain("END|ABORTED|OPERATOR|"));
        }

        static string Execute(bool abort = false)
        {
            var template = SbcToShipTemplate.Convert(new SbcFileReader().ReadFile(SimulationRun.DefaultBlueprint), "flight fixture");
            using (var context = new FleetTestContext(new SimWorld(SePhysicsProfile.Default, Vector3D.Zero)))
            {
                var ship = context.Spawn(template, SpawnState.At(Vector3D.Zero, MatrixD.Identity));
                context.Step(2);
                var pb = (SimProgrammableBlock)ship.TerminalSystem.GetBlockWithName("FC Validation");
                var lcd = (IMyTextPanel)ship.TerminalSystem.GetBlockWithName("FC Report");
                var type = typeof(ValidationRunner).Assembly.GetType("FleetCommand.Validation.FlightProbeRunner");
                Assert.That(type, Is.Not.Null, "The shared ingame flight experiment must exist.");
                var runner = (IValidationExperiment)Activator.CreateInstance(type, ship.TerminalSystem, pb.Runtime, pb, new Action<string>(_ => { }), "");
                pb.RunHandler = arg => runner.Main(arg, pb.CurrentUpdateSource);
                pb.Run("flightprobe", UpdateType.Terminal);
                int ticks = 0;
                while (runner.IsRunning && ticks++ < 18000)
                {
                    context.Step();
                    if (abort && runner.Report.Contains("S|power_full_rest|10|")) pb.Run("abort", UpdateType.Terminal);
                }
                Assert.That(runner.IsRunning, Is.False, "Experiment exceeded bounded runtime.");
                foreach (string axis in new[] { "+X", "-X", "+Y", "-Y", "+Z", "-Z" })
                    Assert.That(((IMyThrust)ship.TerminalSystem.GetBlockWithName("FC Thrust " + axis)).ThrustOverridePercentage, Is.Zero);
                Assert.That(((IMyRemoteControl)ship.TerminalSystem.GetBlockWithName("FC Reference")).IsAutoPilotEnabled, Is.False);
                foreach (string name in new[] { "FC Gyro A", "FC Gyro B" })
                    Assert.That(((IMyGyro)ship.TerminalSystem.GetBlockWithName(name)).GyroOverride, Is.False);
                context.Step(10);
                Assert.That(lcd.GetText(), Is.EqualTo(runner.Report));
                return lcd.GetText();
            }
        }
    }
}
