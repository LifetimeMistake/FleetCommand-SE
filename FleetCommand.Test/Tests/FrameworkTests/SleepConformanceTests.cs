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
