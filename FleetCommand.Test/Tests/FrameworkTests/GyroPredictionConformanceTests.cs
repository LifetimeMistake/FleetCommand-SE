using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Shared;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class GyroPredictionConformanceTests : FleetTestFixture
    {
        [Test]
        public void RepeatingSameOverrideFlagDoesNotRestartAccelerationRamp()
        {
            var ship = Fc.Spawn(new ShipTemplate("override-sync", 2.5f, new[] {
                new TemplateBlock("G", Vector3I.Zero, 1200, 2.5f, kind: BlockKind.Gyro, gyroForce: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var gyro = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            gyro.GyroOverride = true;
            gyro.Pitch = 2;
            Fc.Step();
            int frames = ship.Grid.GyroRateSystem.OverrideRampFrames;
            gyro.GyroOverride = true; // Sync<bool>.SetValue ignores equal values.
            Fc.Step();
            Assert.That(ship.Grid.GyroRateSystem.OverrideRampFrames, Is.EqualTo(frames - 1),
                "An unchanged flag must not fire SetGyroOverride and re-arm the source countdown.");
        }

        [Test]
        public void PredictionUsesRestoredPowerBeforeNextActuatorAggregation()
        {
            var ship = Fc.Spawn(CraftTemplates.PoweredDriverCraft("prediction-power", 1, 100,
                new TemplateBlock("G", new Vector3I(4, 0, 0), 400, 2.5f,
                    kind: BlockKind.Gyro, gyroForce: 10000, gyroPowerDemand: 1)), SpawnState.At(Vector3D.Zero));
            Fc.Step();
            var gyro = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            var battery = (SimBatteryBlock)ship.TerminalSystem.GetBlockWithName("Battery");
            var system = ship.Grid.GyroRateSystem;
            var command = new Vector3(0, 0.5f, 0);
            ship.Grid.Body.SetState(ship.Body.WithAngularVelocity(new Vector3(0, 10, 0)));
            var powered = system.PredictAngularVelocity(command, false);
            battery.Enabled = false;
            Fc.Step(); // Cache an unpowered authority pool.
            Assert.That(gyro.IsPowered, Is.False);
            battery.Enabled = true;
            // Exercise the real phase-4 allocation/working transition, before
            // the phase-5 aggregation that would otherwise hide a stale query.
            ship.Grid.PowerBus.Recompute(1f / 60f);
            gyro.RecomputeIsWorking();
            Assert.That(gyro.IsPowered, Is.True);
            ship.Grid.Body.SetState(ship.Body.WithAngularVelocity(new Vector3(0, 10, 0)));
            var restored = system.PredictAngularVelocity(command, false);
            Assert.That(restored, Is.EqualTo(powered));
            Assert.That(restored.Y, Is.GreaterThan(10));
        }

        [Test]
        public void PredictionDropsBrokenGyroFromCachedAuthority()
        {
            var ship = Fc.Spawn(new ShipTemplate("functional-prediction", 2.5f, new[] {
                new TemplateBlock("A", new Vector3I(-1, 0, 0), 1000, 2.5f, kind: BlockKind.Gyro, gyroForce: 10000),
                new TemplateBlock("B", new Vector3I(1, 0, 0), 1000, 2.5f, kind: BlockKind.Gyro, gyroForce: 10000),
            }), SpawnState.At(Vector3D.Zero).WithAngularVelocity(new Vector3(0, 10, 0)));
            var system = ship.Grid.GyroRateSystem;
            var command = new Vector3(0, 0.5f, 0);
            var before = system.PredictAngularVelocity(command, false);
            ((SimGyro)ship.TerminalSystem.GetBlockWithName("A")).IsFunctional = false;
            var after = system.PredictAngularVelocity(command, false);
            Assert.That(after.Y - 10, Is.EqualTo((before.Y - 10) / 2).Within(0.00001),
                "A broken gyro contributes no authority; physical mass/inertia are unchanged.");
            Assert.That(before.Y, Is.GreaterThan(10));
        }

        [TestCase(1f, 0.1006666667)]
        [TestCase(0.5f, 0.1003333333)]
        public void PredictionUsesInertiaAndCurrentGyroPowerWithoutApplyingImpulse(float power, double expected)
        {
            // A 1200 kg cube, side 2.5 m: inertia=1250, inverse=0.0008.
            // num3=max(1,0.0008*125000)=100. With smoothing off, torque is
            // control(0.5)*force(10000)*power/100; predicted delta is torque*0.0008/60.
            var ship = Fc.Spawn(new ShipTemplate("predictor", 2.5f, new[] {
                new TemplateBlock("G", Vector3I.Zero, 1200, 2.5f, kind: BlockKind.Gyro, gyroForce: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var system = ship.Grid.GyroRateSystem;
            system.Aggregate();
            var gyro = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            gyro.GyroPower = power;
            ship.Grid.Body.SetState(ship.Body.WithAngularVelocity(new Vector3(0, 0.1f, 0)));
            var predicted = system.PredictAngularVelocity(new Vector3(0, 0.5f, 0), false);
            Assert.That(predicted.Y, Is.EqualTo(expected).Within(0.000001));
            Assert.That(predicted.X, Is.Zero);
            Assert.That(predicted.Z, Is.Zero);
            Assert.That(ship.Grid.Body.AngularVelocity.Y, Is.EqualTo(0.1f), "Prediction must not change physics state.");
        }
    }
}
