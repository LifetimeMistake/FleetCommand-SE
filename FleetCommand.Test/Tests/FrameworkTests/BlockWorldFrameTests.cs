using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    public class BlockWorldFrameTests : FleetTestFixture
    {
        [Test]
        public void RotatedBlock_ComposesLocalThenGridLikeGamePositionComponent()
        {
            var local = Quaternion.CreateFromAxisAngle(Vector3.Up, MathHelper.PiOver2);
            var block = new TemplateBlock("block", new Vector3I(2, 1, -3), 20, 0.5f, orientation: local);
            var template = new ShipTemplate("frame", 0.5f, new[] { block });
            var rotation = MatrixD.CreateRotationX(0.6);
            var ship = Fc.Spawn(template, SpawnState.At(new Vector3D(10, 20, 30), rotation));
            var expected = (MatrixD)block.LocalTransform * ship.Grid.WorldMatrix;
            var actual = ship.TerminalSystem.GetBlockWithName("block").WorldMatrix;
            Assert.That(Vector3D.Distance(actual.Translation, expected.Translation), Is.LessThan(1e-6));
            Assert.That(Vector3D.Distance(actual.Forward, expected.Forward), Is.LessThan(1e-6));
        }
    }
}
