using System;
using System.Globalization;
using System.Linq;
using FleetCommand.Validation.Launcher;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Validation.Tests
{
    public class ComparisonTests
    {
        string _report;
        [OneTimeSetUp]
        public void RunBaseline()
        {
            _report = SimulationRun.Execute(SimulationRun.DefaultBlueprint, Vector3D.Zero, MatrixD.Identity);
            Assert.That(_report, Does.Contain("END|COMPLETE|"));
        }

        [Test]
        public void IdenticalCompleteReports_Pass()
        {
            Assert.That(ReportComparison.Compare(_report, _report).Success, Is.True);
        }

        [Test]
        public void RotatedRun_NormalizesFlightProbesAndExposesAutopilotSensitivity()
        {
            string moved = SimulationRun.Execute(SimulationRun.DefaultBlueprint, new Vector3D(12500, -3600, 8100),
                MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2));
            var result = ReportComparison.Compare(_report, moved);
            // The simulator's approximate autopilot currently amplifies small world-frame
            // differences. Keep this visible: don't widen tolerances to call it equivalent.
            Assert.That(result.Success, Is.False);
            Assert.That(result.Differences.All(d => d.StartsWith("waypoint_forward|") || d.StartsWith("connector_unpaired|")),
                Is.True, string.Join("\n", result.Differences.Take(20)));
            Assert.That(result.Differences.Any(d => d.StartsWith("waypoint_forward|")), Is.True);
        }

        [Test]
        public void TranslatedRun_NormalizesTheEntireSuite()
        {
            string moved = SimulationRun.Execute(SimulationRun.DefaultBlueprint, new Vector3D(12500, -3600, 8100), MatrixD.Identity);
            var result = ReportComparison.Compare(_report, moved);
            Assert.That(result.Success, Is.True, string.Join("\n", result.Differences.Take(20)));
        }

        [TestCase("NaN")]
        [TestCase("Infinity")]
        [TestCase("9999.000000")]
        public void NonfiniteOrChangedMeasurement_Fails(string value)
        {
            var lines = _report.Split('\n');
            int index = Array.FindIndex(lines, line => line.StartsWith("S|thrust_px|15|"));
            var cells = lines[index].Split('|');
            cells[8] = value;
            lines[index] = string.Join("|", cells);
            Assert.That(ReportComparison.Compare(_report, string.Join("\n", lines)).Success, Is.False);
        }

        [Test]
        public void MissingCaseOrFooter_FailsEvenWhenBothInputsAreTheSame()
        {
            string truncated = _report.Substring(0, _report.LastIndexOf("END|", StringComparison.Ordinal));
            Assert.That(ReportComparison.Compare(truncated, truncated).Success, Is.False);
            string omitted = string.Join("\n", _report.Split('\n').Where(line => !line.Contains("thrust_px")));
            Assert.That(ReportComparison.Compare(omitted, omitted).Success, Is.False);
        }

        [Test]
        public void IncompatibleFixtureMass_IsRejected()
        {
            var lines = _report.Split('\n');
            int index = Array.FindIndex(lines, line => line.StartsWith("META|mass_kg|"));
            lines[index] = "META|mass_kg|1.000000";
            Assert.That(ReportComparison.Compare(_report, string.Join("\n", lines)).Success, Is.False);
        }

        [Test]
        public void Reports_AreInvariantCulture()
        {
            var saved = CultureInfo.CurrentCulture;
            try
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("pl-PL");
                Assert.That(ValidationReport.Number(1.25), Is.EqualTo("1.250000"));
                Assert.That(ReportComparison.Compare(_report, _report).Success, Is.True);
            }
            finally { CultureInfo.CurrentCulture = saved; }
        }
    }
}
