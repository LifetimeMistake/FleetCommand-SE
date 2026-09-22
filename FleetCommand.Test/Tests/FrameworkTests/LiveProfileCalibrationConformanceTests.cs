using FleetCommand.Test.Framework;
using FleetCommand.Test.SpaceEngineers;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (:462, ): the manual
    /// live-profile calibration record is kept at
    /// <c>docs/superpowers/calibration/se-live-profile-calibration.md</c>.
    /// This test locks the sim's compatibility profile and its
    /// source-derived behaviour constants to that record, so the two cannot
    /// silently drift.
    ///
    /// No live game is available in this environment, so no value is
    /// MEASURED; the record distinguishes SOURCE-DERIVED values (read from
    /// the decompiled game source — asserted here exactly) from
    /// REPRESENTATIVE values (design defaults whose exact live value needs
    /// game calibration — asserted to match the recorded baseline). This is
    /// source-derived conformity, never a check of Havok against itself.
    /// </summary>
    [TestFixture]
    public class LiveProfileCalibrationConformanceTests
    {
        // — source-derived behaviour constants (must equal the game source) —

        [Test]
        public void GyroOverrideRamp_MatchesSourceRecord()
        {
            // MyGridGyroSystem.cs:311-316.
            Assert.That(GyroRateSystem.LargeRateRampFrames, Is.EqualTo(120),
                "Large-rate override ramp frames must match the game source (MyGridGyroSystem.cs:313).");
            Assert.That(GyroRateSystem.RampFrameSqThreshold, Is.EqualTo(2.467401f).Within(1e-6),
                "Ramp frame threshold must match the game source (MyGridGyroSystem.cs:312).");
            Assert.That(GyroRateSystem.RampFrameScale, Is.EqualTo(48.228886f).Within(1e-5),
                "Ramp frame scale must match the game source (MyGridGyroSystem.cs:314).");
        }

        [Test]
        public void GyroTorqueAndSlowdown_Constants_MatchesSourceRecord()
        {
            Assert.That(GyroRateSystem.MaxSlowdown, Is.EqualTo(0.93f).Within(1e-6),
                "MaxSlowdown must match the game (MyGridGyroSystem.cs:23, WELD_LANDING_GEARS ? 0.8 : 0.93).");
            Assert.That(GyroRateSystem.InvTensorMaxLimit, Is.EqualTo(125000f),
                "InvTensorMaxLimit must match the game (MyGridGyroSystem.cs:21).");
            Assert.That(GyroRateSystem.TorqueSqLenThreshold, Is.EqualTo(0.0001f).Within(1e-8),
                "Torque commit threshold must match the game (Torque.LengthSquared() < 0.0001).");
            Assert.That(GyroRateSystem.SlowdownTorqueMultiplierSmall, Is.EqualTo(5f),
                "Small-grid slowdown torque multiplier must match the game (MyFakes).");
            Assert.That(GyroRateSystem.SlowdownTorqueMultiplierLarge, Is.EqualTo(2f),
                "Large-grid slowdown torque multiplier must match the game (MyFakes).");
        }

        // — compatibility profile (locked to the calibration record) —

        [Test]
        public void CompatibilityProfile_MatchesCalibrationRecord()
        {
            SePhysicsProfile p = SePhysicsProfile.Default;
            Assert.That(p.Name, Is.EqualTo("SpaceEngineers-Default"),
                "The active default compatibility profile name must be the recorded profile.");

            // Representative caps (game value comes from the live sector
            // environment; the recorded baseline locks this sim's default).
            Assert.That(p.MaxLinearVelocity, Is.EqualTo(100f).Within(1e-6),
                "Default linear speed cap must match the recorded compatibility baseline.");
            Assert.That(p.MaxAngularVelocity, Is.EqualTo(10f).Within(1e-6),
                "Default angular speed cap must match the recorded compatibility baseline.");

            // Source-aligned: zero damping unless a profile specifies it.
            Assert.That(p.LinearDamping, Is.EqualTo(0f).Within(1e-9),
                "The default profile must apply zero linear damping ().");
            Assert.That(p.AngularDamping, Is.EqualTo(0f).Within(1e-9),
                "The default profile must apply zero angular damping.");
            Assert.That(p.CollisionResponse, Is.EqualTo(true),
                "The default profile must enable collision response.");

            // Connector capture knobs (recorded baseline; exact live values
            // need game calibration per the follow-up in the record).
            Assert.That(p.ConnectorReach, Is.EqualTo(2.5f).Within(1e-6),
                "ConnectorReach must match the recorded calibration baseline.");
            Assert.That(p.ConnectorAngularErrorMax, Is.EqualTo(0.174533f).Within(1e-6),
                "ConnectorAngularErrorMax must match the recorded calibration baseline.");
            Assert.That(p.ConnectorLinearVelocityMax, Is.EqualTo(0.5f).Within(1e-6),
                "ConnectorLinearVelocityMax must match the recorded calibration baseline.");
            Assert.That(p.ConnectorAngularVelocityMax, Is.EqualTo(0.5f).Within(1e-6),
                "ConnectorAngularVelocityMax must match the recorded calibration baseline.");
            Assert.That(p.ConnectorApproachGap, Is.EqualTo(0.2f).Within(1e-6),
                "ConnectorApproachGap must match the recorded calibration baseline.");
            Assert.That(p.ConnectorApproachAngle, Is.EqualTo(0.02f).Within(1e-6),
                "ConnectorApproachAngle must match the recorded calibration baseline.");
            Assert.That(p.AutopilotArrivalDistance, Is.EqualTo(3f).Within(1e-6),
                "AutopilotArrivalDistance must match the recorded calibration baseline.");
        }
    }
}
