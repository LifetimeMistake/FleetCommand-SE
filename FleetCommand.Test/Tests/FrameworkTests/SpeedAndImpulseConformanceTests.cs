using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.SpaceEngineers;
using FleetCommand.Test.Templates;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  conformance (): "linear ... velocity caps" and
    /// "angular impulse response about principal and mixed axes". These close
    /// the physics-conformance gaps in the  list that earlier stages left
    /// for the rigid-body dynamics.
    ///
    /// Linear cap: the per-grid linear-velocity cap set on the Havok body
    /// (<c>MaxLinearVelocity</c> — MyGridPhysics.EnvironmentDefinition
    /// max-speed, expressed here through <see cref="SePhysicsProfile"/>; the
    /// sim also clamps the post-physics grid speed per frame, mirroring the
    /// game's per-frame max-speed clamp). A ship with no autopilot firing
    /// thrust whose unconstrained speed would far exceed the cap must climb
    /// to and hold the cap, never exceed it.
    ///
    /// Angular impulse: applying a known body-axis angular impulse J to a
    /// body must yield the analytic angular velocity Ω = I⁻¹·J (working
    /// from the closed-form box inertia via
    /// <see cref="MassProperties.AnalyticFromBoxes"/> — never from the same
    /// Havok result). About a principal axis Ω is parallel to J; about a
    /// mixed (non-principal) axis the anisotropic inertia deforms the
    /// rotation so Ω is not parallel to J. Both are analytic and falsifiable.
    /// </summary>
    [TestFixture]
    public class SpeedAndImpulseConformanceTests : FleetTestFixture
    {
        // — linear velocity cap (grid physics cap, not the autopilot speed limit) —

        [Test]
        public void LinearSpeedCap_Holds_UnderSustainedThrust()
        {
            // A strong +X engine with no autopilot and no controller under
            // control: the grid must climb to the profile cap and hold it,
            // never exceed it, even though the unconstrained impulse
            // (F·t/m) would far outrun the cap.
            SimShip ship = Fc.Spawn(
                ThrustCraft("cap", maxThrust: 500000f, hullMass: 2000f),
                SpawnState.At(Vector3D.Zero));
            var t = (SimThrust)ship.TerminalSystem.GetBlockWithName("T");
            t.ThrustOverride = 500000f;

            float cap = Fc.World.Profile.MaxLinearVelocity; // 100 from Default
            double saturate = 0d;
            float peak = 0f;
            for (int i = 0; i < 180; i++) // 3 s
            {
                Fc.Step();
                float speed = (float)ship.Body.LinearVelocity.Length();
                if (speed > peak)
                {
                    peak = speed;
                }
                if (speed > cap * 0.95f && saturate == 0f)
                {
                    saturate = Fc.Time;
                }
            }

            Assert.That(peak, Is.LessThanOrEqualTo(cap * 1.02f + 0.05),
                "The grid linear-velocity cap must never be exceeded, even under sustained over-cap thrust.");
            Assert.That(peak, Is.GreaterThan(cap * 0.9f),
                "Sustained over-cap thrust must drive the grid up to (not just toward) the cap.");
        }

        [Test]
        public void LinearSpeedCap_ClampsAnOverCapInitialVelocity()
        {
            // Spawn a ship already moving faster than the cap (a scenario
            // applying an external high-speed state); the first physics
            // interval must clamp it to the cap.
            SimShip ship = Fc.Spawn(
                ThrustCraft("clamp", maxThrust: 0f, hullMass: 2000f),
                SpawnState.At(Vector3D.Zero).WithVelocity(new Vector3D(0, 0, 260f)));
            float cap = Fc.World.Profile.MaxLinearVelocity;

            Fc.Step();

            Assert.That(ship.Body.LinearVelocity.Length(), Is.LessThanOrEqualTo(cap * 1.02f + 0.05),
                "An over-cap initial velocity must be clamped to the grid linear cap on the first step.");
        }

        // — angular impulse response: principal axis —

        [Test]
        public void AngularImpulseAboutPrincipalAxis_RotatesOnlyThatAxis_Analytic()
        {
            // A single centered box: all body axes are principal (I is
            // diagonal). A pure +X body angular impulse must spin only the
            // X axis at Ω = J / Ixx.
            var template = new ShipTemplate("prin", 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, 2000f, 2.5f),
            });
            MassProperties analytic = MassProperties.AnalyticFromBoxes(BoxElementsOf(template));
            float J = 1200f;

            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            Fc.AtTick(0, c => c.ApplyAngularImpulse(ship, new Vector3(J, 0, 0)));
            Fc.Step();

            float omegaX = (float)(J * analytic.InverseInertiaTensor.M11); // 1/Ixx
            Assert.That(ship.Body.AngularVelocity.X, Is.EqualTo(omegaX).Within(Math.Abs(omegaX) * 0.03),
                "An angular impulse about a principal axis must give Ω = J / Ixx from the analytic box inertia.");
            Assert.That(ship.Body.AngularVelocity.Y, Is.EqualTo(0).Within(1e-3),
                "A principal-axis impulse must not spin the orthogonal axes.");
            Assert.That(ship.Body.AngularVelocity.Z, Is.EqualTo(0).Within(1e-3));
        }

        // — angular impulse response: mixed (non-principal) axis —

        [Test]
        public void AngularImpulseAboutMixedAxis_DeformsIntoAnisotropicRotation_Analytic()
        {
            // A hull plus a heavy arm along +X makes I anisotropic
            // (Iyy, Izz >> Ixx). An impulse not aligned with a principal
            // axis then produces an angular velocity whose direction
            // differs from the impulse axis — the inertia tensor
            // deforming the rotation. Expected Ω = I⁻¹·J (analytic).
            var template = new ShipTemplate("mix", 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, 2000f, 2.5f),
                new TemplateBlock("Arm", new Vector3I(4, 0, 0), 1200f, 2.5f),
            });
            MassProperties analytic = MassProperties.AnalyticFromBoxes(BoxElementsOf(template));
            Vector3 J = new Vector3(1400f, 900f, 0f);
            Vector3 expected = MatrixTimesVector(analytic.InverseInertiaTensor, J);

            SimShip ship = Fc.Spawn(template, SpawnState.At(Vector3D.Zero));
            Fc.AtTick(0, c => c.ApplyAngularImpulse(ship, J));
            Fc.Step();

            Vector3 omega = ship.Body.AngularVelocity;
            Assert.That(omega.X, Is.EqualTo(expected.X).Within(Math.Abs(expected.X) * 0.05),
                "Mixed-axis angular impulse, Ω_x = (I⁻¹·J)_x from the analytic anisotropic inertia.");
            Assert.That(omega.Y, Is.EqualTo(expected.Y).Within(Math.Abs(expected.Y) * 0.05),
                "Mixed-axis angular impulse, Ω_y = (I⁻¹·J)_y from the analytic anisotropic inertia.");
            Assert.That(omega.Z, Is.EqualTo(expected.Z).Within(Math.Abs(expected.Z) * 0.05));

            // The deformation is real: Ω must not be parallel to J when
            // the inertia is anisotropic (Ixx != Iyy). The analytic
            // expectation above already encodes this; assert it directly
            // so the test would catch a naive I-scalar implementation.
            Vector3 dirOmega = omega;
            Vector3 dirJ = J;
            double cross = (dirOmega.Cross(dirJ)).Length();
            Assert.That(cross, Is.GreaterThan(1e-3 * dirOmega.Length() * dirJ.Length()),
                "An anisotropic inertia must deflect the angular velocity off the impulse axis (mixed axis).");
        }

        // — helpers —

        private static List<MassProperties.MassElement> BoxElementsOf(ShipTemplate template)
        {
            var boxes = new List<MassProperties.MassElement>();
            foreach (TemplateBlock b in template.Blocks)
            {
                boxes.Add(new MassProperties.MassElement
                {
                    Mass = b.DryMass,
                    HalfExtents = b.HalfExtents,
                    CenterLocal = b.CenterLocal,
                });
                if (b.InventoryMass > 0f)
                {
                    boxes.Add(new MassProperties.MassElement
                    {
                        Mass = b.InventoryMass,
                        HalfExtents = b.HalfExtents,
                        CenterLocal = b.CenterLocal,
                    });
                }
            }
            return boxes;
        }

        private static Vector3 MatrixTimesVector(Matrix m, Vector3 v)
        {
            return new Vector3(
                m.M11 * v.X + m.M12 * v.Y + m.M13 * v.Z,
                m.M21 * v.X + m.M22 * v.Y + m.M23 * v.Z,
                m.M31 * v.X + m.M32 * v.Y + m.M33 * v.Z);
        }

        /// <summary>A hull plus one +X thruster (forward = -X pushes +X).</summary>
        private static ShipTemplate ThrustCraft(string name, float maxThrust, float hullMass)
        {
            return new ShipTemplate(name, 2.5f, new[]
            {
                new TemplateBlock("Hull", Vector3I.Zero, hullMass, 2.5f),
                new TemplateBlock("T", new Vector3I(2, 0, 0), 100f, 2.5f,
                    kind: BlockKind.FixedThruster, thrustDirection: Vector3I.Left, maxThrust: maxThrust),
            });
        }
    }
}
