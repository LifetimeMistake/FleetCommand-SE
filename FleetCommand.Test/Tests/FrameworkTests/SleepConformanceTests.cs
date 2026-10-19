using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Havok;
using NUnit.Framework;
using VRageMath;
using FleetCommand.Test.SpaceEngineers;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class SleepConformanceTests : FleetTestFixture
    {
        [Test]
        public void DampenerWrite_WithoutThrustComponentDoesNotWakeGrid()
        {
            var ship = Fc.Spawn(new ShipTemplate("no-thrust-component", 0.5f, new[] {
                new TemplateBlock("Pilot", Vector3I.Zero, 1000, 0.5f, kind: BlockKind.ShipController),
            }), SpawnState.At(Vector3D.Zero));
            Fc.Step(600);
            Assert.That(ship.Grid.Body.IsActive, Is.False);
            ship.Controller.DampenersOverride = false;
            Fc.Step();
            Assert.That(ship.Grid.Body.IsActive, Is.False,
                "EnableDampingInternal returns when the grid has no thrust component.");
        }

        [TestCase(false, true)]
        [TestCase(true, true)]
        [TestCase(false, false)]
        [TestCase(true, false)]
        public void DampenerWrite_WakesSleepingGridOnlyWhenValueChanges(bool value, bool changed)
        {
            var ship = Fc.Spawn(new ShipTemplate("dampener-wake", 0.5f, new[] {
                new TemplateBlock("Pilot", Vector3I.Zero, 1000, 0.5f, kind: BlockKind.ShipController),
                new TemplateBlock("Engine", new Vector3I(2, 0, 0), 1000, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var pilot = (SimShipController)ship.Controller;
            pilot.DampenersOverride = changed ? !value : value;
            Fc.Step(600);
            Assert.That(ship.Grid.Body.IsActive, Is.False);

            pilot.DampenersOverride = value;
            Fc.Step();

            Assert.That(ship.Grid.Body.IsActive, Is.EqualTo(changed),
                "MyCubeGrid.EnableDampingInternal wakes physics on a transition and returns immediately on an equal write.");
        }

        [TestCase(false)]
        [TestCase(true)]
        public void ThrusterFunctionalChange_WakesSleepingGridEvenWhenDisabled(bool restored)
        {
            var ship = Fc.Spawn(new ShipTemplate("functional-wake", 0.5f, new[] {
                new TemplateBlock("Engine", Vector3I.Zero, 1000, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var engine = (SimThrust)ship.TerminalSystem.GetBlockWithName("Engine");
            engine.Enabled = false;
            engine.IsFunctional = !restored;
            Fc.Step(600);
            Assert.That(ship.Grid.Body.IsActive, Is.False);

            engine.IsFunctional = restored;
            Fc.Step();

            Assert.That(ship.Grid.Body.IsActive, Is.True,
                "MyThrusterBlockThrustComponent.ComponentStack_IsFunctionalChanged wakes physics independently of IsWorking.");
            Assert.That(engine.IsWorking, Is.False);
            Assert.That(ship.Body.LinearVelocity.Length(), Is.Zero);
        }

        [Test]
        public void PoweredGyroOverride_DisablesAutomaticDeactivationUntilReleased()
        {
            var ship = Fc.Spawn(new ShipTemplate("gyro-deactivation", 0.5f, new[] {
                new TemplateBlock("G", Vector3I.Zero, 1000, 0.5f, kind: BlockKind.Gyro, gyroForce: 10000),
            }), SpawnState.At(Vector3D.Zero));
            var gyro = (SimGyro)ship.TerminalSystem.GetBlockWithName("G");
            gyro.GyroOverride = true;
            gyro.Yaw = 0.1f;
            Fc.Step();
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.EnableDeactivation, Is.False,
                    "MyGridGyroSystem.UpdateAutomaticDeactivation suppresses sleep for a powered nonzero override.");
            gyro.GyroOverride = false;
            Fc.Step();
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.EnableDeactivation, Is.True);
        }

        [Test]
        public void PilotInput_WakesSleepingSmallGrid()
        {
            var ship = Fc.Spawn(new ShipTemplate("pilot-wake", 0.5f, new[] {
                new TemplateBlock("Pilot", Vector3I.Zero, 1000, 0.5f, kind: BlockKind.ShipController),
                new TemplateBlock("Engine", new Vector3I(2, 0, 0), 1000, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 10000),
            }), SpawnState.At(Vector3D.Zero));
            Fc.Step(600);
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.IsActive, Is.False);
            ((SimShipController)ship.Controller).IsUnderControl = true;
            ((SimShipController)ship.Controller).MoveIndicator = Vector3.UnitX;
            Fc.Step();
            Assert.That(ship.Body.LinearVelocity.X, Is.GreaterThan(0.01));
        }

        [TestCase(0.5f, HkSolverDeactivation.Low)]
        [TestCase(2.5f, HkSolverDeactivation.Off)]
        public void Grid_UsesGameSizeDependentSolverDeactivation(float gridSize, HkSolverDeactivation expected)
        {
            var template = new ShipTemplate("sleep", gridSize, new[] {
                new TemplateBlock("Hull", Vector3I.Zero, 1000, gridSize),
            });
            var ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.Motion.GetDeactivationClass(), Is.EqualTo(expected));
        }

        [Test]
        public void ZeroThrust_DoesNotKeepSmallGridAwake()
        {
            var ship = Fc.Spawn(new ShipTemplate("sleep", 0.5f, new[] {
                new TemplateBlock("Hull", Vector3I.Zero, 1000, 0.5f),
            }), SpawnState.At(Vector3D.Zero));
            Fc.Step(600);
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.IsActive, Is.False);
        }

        [Test]
        public void ThrustForce_IsIgnoredForSleepingBody()
        {
            var ship = Fc.Spawn(new ShipTemplate("active-only", 0.5f, new[] {
                new TemplateBlock("Engine", Vector3I.Zero, 1000, 0.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: 10000),
            }), SpawnState.At(Vector3D.Zero));
            Fc.Step(600); // Let Havok deactivate naturally; Deactivate() queues a request.
            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                Assert.That(ship.Grid.Body.Body.IsActive, Is.False);
            ship.Grid.Body.ApplyThrustForce(new Vector3(10000, 0, 0), 1f / 60f);
            Fc.Step();
            Assert.That(ship.Body.LinearVelocity.Length(), Is.Zero);
        }
    }
}
