using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>Conformance of the controller's rotation control: RotationIndicator/RollIndicator rotate the grid via the game's MoveAndRotate path.</summary>
    [TestFixture]
    public class RotationControlConformanceTests : FleetTestFixture
    {
        [Test]
        public void Rotation_ControlTorque_MatchesMoveAndRotate_Formula()
        {
            SimShip ship = Fc.Spawn(
                RotationCraft("rot", hullMass: 5000f, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero));
            SimShipController ctrl = Control(ship);
            ctrl.IsUnderControl = true;

            // RotationIndicator=(pitch=0, yaw=4): /20 -> (0, 0.2); the -
            // signs give ControlTorque = (0, -0.2, 0) for an identity mount.
            ctrl.RotationIndicator = new Vector2(0f, 4f);
            Fc.Step();

            Vector3 ct = ship.Grid.RotationControl.LastControlTorque;
            Assert.That(ship.Grid.RotationControl.RotationActive, Is.True,
                "An under-control controller with ControlGyros must drive the rotation.");
            Assert.That(ct.X, Is.EqualTo(0f).Within(1e-5), "Pitch 0 -> no X control torque.");
            Assert.That(ct.Y, Is.EqualTo(-0.2f).Within(1e-5), "Yaw must scale by /20 and negate (yaw=4 -> -0.2).");
            Assert.That(ct.Z, Is.EqualTo(0f).Within(1e-5), "No roll -> no Z control torque.");

            // Roll: *0.2, negated; RotationIndicator=(0,0) + Roll=2 -> -0.4 on Z.
            ctrl.RotationIndicator = Vector2.Zero;
            ctrl.RollIndicator = 2f;
            Fc.Step();
            Assert.That(ship.Grid.RotationControl.LastControlTorque.Z, Is.EqualTo(-0.4f).Within(1e-5),
                "Roll must scale by *0.2 and negate (roll=2 -> -0.4 on Z).");
        }

        [Test]
        public void Rotation_SphereClamp_And_ControlGyrosGate()
        {
            SimShip ship = Fc.Spawn(
                RotationCraft("clamp", hullMass: 5000f, gyroForce: 4e6f),
                SpawnState.At(Vector3D.Zero));
            SimShipController ctrl = Control(ship);
            ctrl.IsUnderControl = true;

            // |RotationIndicator|/20 beyond 1 -> the ClampToSphere(1) must cap it.
            ctrl.RotationIndicator = new Vector2(40f, 0f); // /20 = 2 -> clamp to 1
            Fc.Step();
            Assert.That(ship.Grid.RotationControl.LastControlTorque.X, Is.EqualTo(-1f).Within(1e-6),
                "A rotation command beyond the unit sphere must be clamped to 1 (negated).");

            // ControlGyros off gates the whole rotation block (game's m_controlGyros).
            ctrl.ControlGyros = false;
            ctrl.RotationIndicator = new Vector2(4f, 0f);
            Fc.Step();
            Assert.That(ship.Grid.RotationControl.RotationActive, Is.False,
                "With ControlGyros off the controller must not drive the rotation.");
            Assert.That(ship.Grid.RotationControl.LastControlTorque.LengthSquared(), Is.EqualTo(0).Within(1e-9),
                "With ControlGyros off the control torque must be zero.");
        }

        [Test]
        public void Rotation_WritingRotationIndicator_SpinsTheGrid_EndToEnd()
        {
            SimShip ship = Fc.Spawn(
                RotationCraft("spin", hullMass: 5000f, gyroForce: 8e6f),
                SpawnState.At(Vector3D.Zero));
            SimShipController ctrl = Control(ship);
            ctrl.IsUnderControl = true;

            // A yaw command: negative grid-Y control torque must rotate the
            // grid about -Y (positive yaw input -> the convention locks the
            // sign end-to-end all the way to body angular velocity).
            ctrl.RotationIndicator = new Vector2(0f, 10f); // -> CT = (0, -0.5, 0)
            Fc.RunFor(1.0);

            Vector3 w = ship.Grid.Body.AngularVelocity;
            Assert.That(Math.Abs(w.Y), Is.GreaterThan(0.05),
                "Writing RotationIndicator must actually spin the grid about the commanded axis (end-to-end).");
            Assert.That(w.Y, Is.LessThan(0), "Positive yaw input must rotate the grid about -Y (sign lock).");
            Assert.That(Math.Abs(w.X), Is.LessThan(0.05), "Pure yaw must not rotate about X.");
            Assert.That(Math.Abs(w.Z), Is.LessThan(0.05), "Pure yaw must not rotate about Z.");
        }

        [Test]
        public void Rotation_NoUnderControl_DrivesNothing()
        {
            SimShip ship = Fc.Spawn(
                RotationCraft("coast", hullMass: 5000f, gyroForce: 8e6f),
                SpawnState.At(Vector3D.Zero));
            // Command set but the controller is NOT under control: the
            // rotation block (gated on under-control) must not drive.
            Control(ship).RotationIndicator = new Vector2(0f, 10f); // would command -Y
            Fc.RunFor(1.0);
            Assert.That(ship.Grid.Body.AngularVelocity.Length(), Is.LessThan(1e-3),
                "Without an under-control controller the written RotationIndicator must not rotate the grid.");
        }

        // — helpers —

        private static SimShipController Control(SimShip ship)
        {
            return (SimShipController)ship.Controller;
        }

        private static ShipTemplate RotationCraft(string name, float hullMass, float gyroForce)
        {
            return new ShipTemplate(name, 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 2.5f),
                new TemplateBlock("Pilot", new Vector3I(0, 0, 1), 400f, 2.5f, kind: BlockKind.ShipController),
                new TemplateBlock("G", new Vector3I(2, 0, 0), 400f, 2.5f, kind: BlockKind.Gyro, gyroForce: gyroForce),
            }, maxAngularVelocity: 12f);
        }
    }
}
