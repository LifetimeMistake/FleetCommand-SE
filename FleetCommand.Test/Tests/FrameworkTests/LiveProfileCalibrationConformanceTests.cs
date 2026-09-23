using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.SpaceEngineers;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Locks source-derived behaviour constants and the compatibility profile to the
    /// live-profile calibration record so neither can silently drift (source-derived values
    /// asserted exactly; representative values matched to the recorded baseline).</summary>
    [TestFixture]
    public class LiveProfileCalibrationConformanceTests
    {
        // — source-derived behaviour constants (must equal the game source) —

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

            // Angular caps are the game's per-grid-size limits, effectively
            // non-binding under gyro command (MyGridPhysics.cs:158-159, 382-394).
            Assert.That(p.MaxLinearVelocity, Is.EqualTo(100f).Within(1e-6),
                "Default linear speed cap must match the recorded compatibility baseline.");
            Assert.That(p.MaxAngularVelocityLarge, Is.EqualTo(100f * (float)Math.PI).Within(1e-4),
                "Default large-grid angular speed cap must match the game (ToRadians(18000)).");
            Assert.That(p.MaxAngularVelocitySmall, Is.EqualTo(200f * (float)Math.PI).Within(1e-4),
                "Default small-grid angular speed cap must match the game (ToRadians(36000)).");

            // Source-aligned: zero linear damping; angular damping is the game's 0.1
            // (MyPerGameSettings.DefaultAngularDamping, MyPerGameSettings.cs:137).
            Assert.That(p.LinearDamping, Is.EqualTo(0f).Within(1e-9),
                "The default profile must apply zero linear damping.");
            Assert.That(p.AngularDamping, Is.EqualTo(0.1f).Within(1e-9),
                "The default profile must apply the game's 0.1 angular damping.");
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
