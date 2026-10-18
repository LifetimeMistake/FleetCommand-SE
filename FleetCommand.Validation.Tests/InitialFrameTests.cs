using FleetCommand.Validation;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Validation.Tests
{
    public class InitialFrameTests
    {
        [Test]
        public void Frame_IsInvariantUnderRigidChangeOfWorldCoordinates()
        {
            var rotation = MatrixD.CreateFromYawPitchRoll(0.7, -0.3, 1.2);
            var origin = new Vector3D(82345, -28563, 77563);
            rotation.Translation = origin;
            var frame = new InitialFrame(rotation, origin);
            var local = new Vector3D(4, 5, -6);
            Assert.That(Vector3D.Distance(frame.Position(Vector3D.Transform(local, rotation)), local), Is.LessThan(1e-8));
            Assert.That(Vector3D.Distance(frame.Vector(Vector3D.TransformNormal(local, rotation)), local), Is.LessThan(1e-8));
            var attitude = frame.Attitude(rotation);
            Assert.That(attitude.W, Is.EqualTo(1).Within(1e-10));
            Assert.That(attitude.X * attitude.X + attitude.Y * attitude.Y + attitude.Z * attitude.Z, Is.LessThan(1e-15));
            var turn = MatrixD.CreateRotationY(0.4);
            var relative = frame.Attitude(turn * rotation);
            var expected = QuaternionD.CreateFromRotationMatrix(turn);
            Assert.That(relative.Y, Is.EqualTo(expected.Y).Within(1e-10));
            Assert.That(relative.W, Is.EqualTo(expected.W).Within(1e-10));
        }
    }
}
