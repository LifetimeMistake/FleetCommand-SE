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
        [Test]
        public void CurrentReports_RequireInitialOrientationEvenWhenBothCopiesOmitIt()
        {
            string missing = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("META|initial_q")));
            Assert.That(ReportComparison.Compare(missing, missing).Success, Is.False);
        }

        [Test]
        public void LegacyV4WithoutOrientation_RemainsReadableButCannotMixWithCurrentReports()
        {
            string legacy = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("META|initial_q")))
                .Replace(ValidationRunner.Header, "FCVALIDATION|1|craft-v1|suite-v4");
            Assert.That(ReportComparison.Compare(legacy, legacy).Success, Is.True);
            Assert.That(ReportComparison.Compare(legacy, _report).Success, Is.False);
            Assert.That(ReportComparison.Compare(_report, legacy).Success, Is.False);
        }

        [Test]
        public void SuiteRevisions_CannotBeComparedAsEquivalent_ButLegacyReportsRemainReadable()
        {
            const string oldHeader = "FCVALIDATION|1|craft-v1|suite-v1";
            const string newHeader = "FCVALIDATION|1|craft-v1|suite-v2";
            string legacy = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("D|") && !l.StartsWith("DCOLUMNS|")
                && !l.StartsWith("GATE|") && !l.StartsWith("GATECOLUMNS|") && !l.StartsWith("GATEPOLICY|") && !l.StartsWith("TRACE|")
                && !l.StartsWith("W|") && !l.StartsWith("WPCOLUMNS|")));
            string oldReport = legacy.Replace(ValidationRunner.Header, oldHeader);
            string newReport = legacy.Replace(ValidationRunner.Header, newHeader);
            Assert.That(ReportComparison.Compare(newReport, newReport).Success, Is.True);
            Assert.That(ReportComparison.Compare(oldReport, oldReport).Success, Is.True);
            Assert.That(ReportComparison.Compare(oldReport, newReport).Success, Is.False);
            Assert.That(ReportComparison.Compare(_report, newReport).Success, Is.False);
            string v3 = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("W|") && !l.StartsWith("WPCOLUMNS|")))
                .Replace(ValidationRunner.Header, "FCVALIDATION|1|craft-v1|suite-v3");
            Assert.That(ReportComparison.Compare(v3, v3).Success, Is.True);
            Assert.That(ReportComparison.Compare(_report, v3).Success, Is.False);
        }

        [Test]
        public void DiagnosticCorruption_OrMissingSettlingEvidenceFails()
        {
            var lines = _report.Split('\n');
            int row = Array.FindIndex(lines, l => l.StartsWith("D|"));
            var cells = lines[row].Split('|'); cells[6] = "NaN"; lines[row] = string.Join("|", cells);
            string malformed = string.Join("\n", lines);
            Assert.That(ReportComparison.Compare(malformed, malformed).Success, Is.False);
            string missingGate = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("GATE|thrust_py|")));
            Assert.That(ReportComparison.Compare(missingGate, missingGate).Success, Is.False);
            string truncated = _report.Replace("|dropped|0|", "|dropped|1|");
            Assert.That(ReportComparison.Compare(truncated, truncated).Success, Is.False);
            string missingWaypoint = string.Join("\n", _report.Split('\n').Where(l => !l.StartsWith("W|20|")));
            Assert.That(ReportComparison.Compare(missingWaypoint, missingWaypoint).Success, Is.False);
        }

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
        public void RotatedRun_RejectsDifferentWorldOrientationBeforeComparingPhysics()
        {
            string moved = SimulationRun.Execute(SimulationRun.DefaultBlueprint, new Vector3D(12500, -3600, 8100),
                MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2));
            var result = ReportComparison.Compare(_report, moved);
            Assert.That(result.Success, Is.False,
                "Native constrained low-speed stabilization depends on world axes; relative coordinates cannot remove that physical difference.");
            Assert.That(result.Differences.Single(), Does.Contain("world orientation"));
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

        [TestCase("pl-PL")]
        [TestCase("en-US")]
        [TestCase("de-DE")]
        [TestCase("fr-FR")]
        public void Reports_AreInvariantCulture(string culture)
        {
            var saved = CultureInfo.CurrentCulture;
            try
            {
                CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo(culture);
                Assert.That(ValidationReport.Number(1.25), Is.EqualTo("1.250000"));
                Assert.That(ValidationReport.Number(-1234.5), Is.EqualTo("-1234.500000"));
                Assert.That(ValidationReport.Number(0), Is.EqualTo("0.000000"));
                Assert.That(ReportComparison.Compare(_report, _report).Success, Is.True);
            }
            finally { CultureInfo.CurrentCulture = saved; }
        }
    }
}
