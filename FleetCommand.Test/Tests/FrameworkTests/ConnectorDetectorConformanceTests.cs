using FleetCommand.Test.Physics;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Shared;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    [TestFixture]
    public class ConnectorDetectorConformanceTests : FleetTestFixture
    {
        [Test]
        public void Teleport_AlignsRotatedDetectorBodiesWithAnOffsetCenterOfMass()
        {
            var template = new ShipTemplate("teleport-detectors", 0.5f, new[] {
                new TemplateBlock("Hull", Vector3I.Zero, 7500, 0.5f),
                new TemplateBlock("Connector", new Vector3I(2, 1, 0), 50, 0.5f,
                    orientation: Quaternion.CreateFromAxisAngle(Vector3.Up, 1.2f), kind: BlockKind.Connector)
            });
            var ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            var target = SimRigidBodyState.At(new Vector3D(150, -200, 300),
                QuaternionD.CreateFromRotationMatrix(MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2)));
            ship.Grid.Body.SetState(target);
            Fc.Step(3);
            Assert.That(Vector3D.Distance(ship.Grid.Body.PositionWorld, target.PositionWorld), Is.LessThan(0.001));
            Assert.That(ship.Grid.Body.LinearVelocity.Length(), Is.LessThan(0.001));
            Assert.That(ship.Grid.Body.AngularVelocity.Length(), Is.LessThan(0.001));
        }

        [TestCase(false)]
        [TestCase(true)]
        public void DirectDisposal_ReleasesDetectorConstraintsBeforeTheGridBody(bool disposeGrid)
        {
            var template = new ShipTemplate("dispose-detectors", 0.5f, new[] {
                new TemplateBlock("Hull", Vector3I.Zero, 7500, 0.5f),
                new TemplateBlock("Connector", new Vector3I(2, 0, 0), 50, 0.5f, kind: BlockKind.Connector)
            });
            var ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            var connector = (SimShipConnector)ship.TerminalSystem.GetBlockWithName("Connector");
            if (disposeGrid) { ship.Grid.Dispose(); ship.Grid.Dispose(); }
            else
            {
                connector.Dispose(); connector.Dispose();
                Assert.That(ship.Grid.Body.Body.HasConstraints(), Is.False);
            }
            Fc.Step(2);
        }

        [Test]
        public void UnconnectedConnector_HasInternalConstraintsEvenWhenDisabled_AndRemovesThemWithBlock()
        {
            var template = new ShipTemplate("detector-lifecycle", 0.5f, new[] {
                new TemplateBlock("Hull", Vector3I.Zero, 7500, 0.5f),
                new TemplateBlock("Connector", new Vector3I(2, 0, 0), 50, 0.5f, kind: BlockKind.Connector)
            });
            var ship = Fc.Spawn(template, SpawnState.At(new Vector3D(100, 200, -300), MatrixD.CreateRotationY(0.7)));
            var connector = (SimShipConnector)ship.TerminalSystem.GetBlockWithName("Connector");
            Assert.That(connector.OtherConnector, Is.Null);
            Assert.That(ship.Grid.Body.Body.HasConstraints(), Is.True,
                "MyShipConnector.CreateBodyConstraint fixes its own two detector bodies to the grid, independent of docking.");
            connector.Enabled = false;
            Fc.Step(2);
            Assert.That(ship.Grid.Body.Body.HasConstraints(), Is.True);
            ship.Grid.RemoveBlock(connector);
            Assert.That(ship.Grid.Body.Body.HasConstraints(), Is.False);
            Fc.Step(2);
        }
    }
}
