using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (): "actual VRageMath body/world
    /// transform and handedness checks". The framework reimplements no vector
    /// or matrix types () — it uses the real VRageMath assemblies —
    /// so this locks that the body's world transform is a proper,
    /// right-handed, orthonormal rotation (determinant +1), that it maps
    /// body axes to world axes (body→world = R) and back (world→body = Rᵀ),
    /// and that the body-state pose round-trips through SetState/GetState.
    /// These are algebraic facts about the transform — analytic, never
    /// derived from a same-source Havok observation.
    /// </summary>
    [TestFixture]
    public class HandednessConformanceTests : FleetTestFixture
    {
        [Test]
        public void BodyWorldMatrix_IsOrthonormal_RightHanded()
        {
            QuaternionD rot = QuaternionD.CreateFromAxisAngle(
                Vector3D.Normalize(new Vector3D(1, -2, 3)), 0.83);
            SimShip ship = Fc.Spawn(
                BoxTemplate("hand"),
                SpawnState.At(new Vector3D(12, 5, -7), rot));

            MatrixD m = ship.Grid.WorldMatrix;
            // The rotation part is the upper-left 3x3 (columns are body
            // axes in world coordinates; translation is the body origin).
            double det = Det3x3(m);

            Assert.That(det, Is.EqualTo(1.0).Within(1e-6),
                "The body world rotation must be a proper (right-handed) rotation with determinant +1.");

            // Orthonormal: R · Rᵀ = I (columns are mutually orthogonal unit vectors).
            MatrixD rRt = m * MatrixD.Transpose(m);
            Assert.That(rRt.M11, Is.EqualTo(1.0).Within(1e-4));
            Assert.That(rRt.M22, Is.EqualTo(1.0).Within(1e-4));
            Assert.That(rRt.M33, Is.EqualTo(1.0).Within(1e-4));
            Assert.That(rRt.M12, Is.EqualTo(0.0).Within(1e-4));
            Assert.That(rRt.M13, Is.EqualTo(0.0).Within(1e-4));
            Assert.That(rRt.M21, Is.EqualTo(0.0).Within(1e-4));
            Assert.That(rRt.M23, Is.EqualTo(0.0).Within(1e-4));
            Assert.That(rRt.M31, Is.EqualTo(0.0).Within(1e-4));
            Assert.That(rRt.M32, Is.EqualTo(0.0).Within(1e-4));
        }

        [Test]
        public void BodyWorldColumns_Form_ARightHandedOrthonormalFrame()
        {
            // The body's world rotation columns are the body axes expressed
            // in world coordinates. For a proper (right-handed) rotation
            // they must be a mutually orthogonal set of unit vectors with
            // Right × Up = Forward (the right-hand rule), i.e. the body
            // axes form a right-handed frame in world space. This is the
            // transform contract the autopilot signs rely on (CLAUDE.md:
            // "rotation part is body-to-world transform R; columns are
            // body axes in world coords"), checked at the matrix level.
            QuaternionD rot = QuaternionD.CreateFromAxisAngle(
                Vector3D.Normalize(new Vector3D(0, 2, 1)), 1.1);
            SimShip ship = Fc.Spawn(
                BoxTemplate("inv"),
                SpawnState.At(new Vector3D(0, 0, 0), rot));

            MatrixD r = ship.Grid.WorldMatrix;
            // The rotation's columns are the body axes in world coords;
            // read them from the raw row-major elements (the Right/Up/
            // Forward properties carry VRage's own -possibly negated-
            // axis semantics, e.g. Forward = -Z column, so use the
            // columns directly for an unambiguous frame check).
            Vector3D colX = new Vector3D(r.M11, r.M21, r.M31);
            Vector3D colY = new Vector3D(r.M12, r.M22, r.M32);
            Vector3D colZ = new Vector3D(r.M13, r.M23, r.M33);

            // Unit columns.
            Assert.That(colX.Length(), Is.EqualTo(1.0).Within(1e-4), "R column X (body +X in world) must be a unit vector.");
            Assert.That(colY.Length(), Is.EqualTo(1.0).Within(1e-4), "R column Y must be a unit vector.");
            Assert.That(colZ.Length(), Is.EqualTo(1.0).Within(1e-4), "R column Z must be a unit vector.");

            // Mutually orthogonal.
            Assert.That(Vector3D.Dot(colX, colY), Is.EqualTo(0).Within(1e-4), "Body X and Y axes must be orthogonal.");
            Assert.That(Vector3D.Dot(colX, colZ), Is.EqualTo(0).Within(1e-4), "Body X and Z axes must be orthogonal.");
            Assert.That(Vector3D.Dot(colY, colZ), Is.EqualTo(0).Within(1e-4), "Body Y and Z axes must be orthogonal.");

            // Right-handed: colX × colY = colZ (true for every proper
            // rotation — the identity has (1,0,0) × (0,1,0) = (0,0,1);
            // det = +1 here is confirmed separately in the first test).
            Vector3D cross = Vector3D.Cross(colX, colY);
            AssertVector3D(cross, colZ, 1e-4,
                "The body axes must form a right-handed frame: column X × column Y = column Z.");
        }

        [Test]
        public void BodyState_PoseRoundTrips_ThroughSetAndGet()
        {
            QuaternionD rot = QuaternionD.CreateFromAxisAngle(
                Vector3D.Normalize(new Vector3D(1, 1, 1)), 0.5);
            Vector3D pos = new Vector3D(3, -2, 9);
            Vector3D vel = new Vector3D(1, 0, 0);
            SimShip ship = Fc.Spawn(
                BoxTemplate("round"),
                SpawnState.At(pos, rot).WithVelocity(vel));

            SimRigidBodyState state = ship.Grid.Body.GetState();

            AssertVector3D(state.PositionWorld, pos, 1e-6, "GetState must return the spawned position.");
            AssertVector3D(state.LinearVelocity, vel, 1e-6, "GetState must return the spawned velocity.");
            // Orientation must round-trip through the quaternion: the
            // rotation recovered from GetState equals the spawned one.
            MatrixD r = MatrixD.CreateFromQuaternion(state.Orientation);
            MatrixD expect = MatrixD.CreateFromQuaternion(rot);
            AssertMatrixClose(r, expect, 1e-5, "GetState must return an orientation equal to the spawned rotation.");
        }

        // — helpers —

        private static ShipTemplate BoxTemplate(string name)
        {
            return new ShipTemplate(name, 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, 1000f, 2.5f),
            });
        }

        private static double Det3x3(MatrixD m)
        {
            double a = m.M11, b = m.M12, c = m.M13;
            double d = m.M21, e = m.M22, f = m.M23;
            double g = m.M31, h = m.M32, i = m.M33;
            return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        }

        private static void AssertVector3D(Vector3D actual, Vector3D expected, double tolerance, string message)
        {
            Assert.That((actual - expected).Length(), Is.LessThan(tolerance), message);
        }

        private static void AssertMatrixClose(MatrixD actual, MatrixD expected, double tolerance, string message)
        {
            Assert.That((actual.M11 - expected.M11), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M11)");
            Assert.That((actual.M22 - expected.M22), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M22)");
            Assert.That((actual.M33 - expected.M33), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M33)");
            Assert.That((actual.M12 - expected.M12), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M12)");
            Assert.That((actual.M13 - expected.M13), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M13)");
            Assert.That((actual.M21 - expected.M21), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M21)");
            Assert.That((actual.M23 - expected.M23), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M23)");
            Assert.That((actual.M31 - expected.M31), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M31)");
            Assert.That((actual.M32 - expected.M32), Is.LessThan(tolerance) & Is.GreaterThan(-tolerance), message + " (M32)");
        }
    }
}
