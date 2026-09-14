using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;
using Havok;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    ///  mass-property conformance (, ): the rigid
    /// body's mass, center of mass, and inertia — combined by
    /// <see cref="HkInertiaTensorComputer"/> exactly as the game combines
    /// block mass elements — are validated against independent analytic box
    /// formulas and the parallel-axis theorem. Analytic expectations are
    /// never derived from the Havok result being checked (no circular
    /// validation).
    /// </summary>
    [TestFixture]
    public class RigidBodyMassPropertiesTests : FleetTestFixture
    {
        // Large-grid single-block half extents (grid size 2.5, 1x1x1 block).
        private static readonly Vector3 BlockHalf = new Vector3(1.25f, 1.25f, 1.25f);

        [Test]
        public void SingleCentralBox_HavokMatchesAnalytic()
        {
            var boxes = new[]
            {
                new MassProperties.MassElement
                {
                    Mass = 100f,
                    HalfExtents = BlockHalf,
                    CenterLocal = Vector3.Zero,
                },
            };

            MassProperties havok = HavokFromBoxes(boxes);
            MassProperties analytic = MassProperties.AnalyticFromBoxes(boxes);

            Assert.That(havok.TotalMass, Is.EqualTo(analytic.TotalMass).Within(1e-2),
                "Combined mass must match the template's total block mass.");
            AssertVector3(havok.CenterOfMass, analytic.CenterOfMass, 1e-2,
                "Center of mass of a central block must be the block center.");
            AssertInertia(havok.InertiaTensor, analytic.InertiaTensor, 1e-2,
                "Box inertia about the center must match the analytic box formula.");
        }

        [Test]
        public void ParallelAxis_OffCenterBlocks_HavokMatchesAnalytic()
        {
            // Two identical blocks placed symmetrically about the origin: the
            // combined COM is the origin and each block's inertia is
            // parallel-axis-transferred to it. This exercises the game's
            // combine path with non-trivial offsets.
            var boxes = new[]
            {
                new MassProperties.MassElement { Mass = 50f, HalfExtents = BlockHalf, CenterLocal = new Vector3(-2.5f, 0, 0) },
                new MassProperties.MassElement { Mass = 50f, HalfExtents = BlockHalf, CenterLocal = new Vector3(2.5f, 0, 0) },
            };

            MassProperties havok = HavokFromBoxes(boxes);
            MassProperties analytic = MassProperties.AnalyticFromBoxes(boxes);

            Assert.That(havok.TotalMass, Is.EqualTo(analytic.TotalMass).Within(1e-2));
            AssertVector3(havok.CenterOfMass, analytic.CenterOfMass, 1e-2,
                "Symmetric pair's COM must be the origin.");
            AssertInertia(havok.InertiaTensor, analytic.InertiaTensor, 1e-2,
                "Parallel-axis transferred inertia must match the analytic transfer.");
        }

        [Test]
        public void AsymmetricMass_HavokMatchesAnalytic()
        {
            // Deliberately asymmetric distribution (including inventory mass
            // on one block) so COM is off-origin and off-diagonal inertia
            // terms are non-trivial.
            var boxes = new[]
            {
                new MassProperties.MassElement { Mass = 100f, HalfExtents = BlockHalf, CenterLocal = Vector3.Zero },
                new MassProperties.MassElement { Mass = 200f, HalfExtents = BlockHalf, CenterLocal = new Vector3(4f, 1f, -2f) },
                new MassProperties.MassElement { Mass = 50f, HalfExtents = BlockHalf, CenterLocal = new Vector3(-2f, -1f, 3f) },
                // Inventory mass as a second element at the same volume,
                // mirroring CollectBlockInventories.
                new MassProperties.MassElement { Mass = 30f, HalfExtents = BlockHalf, CenterLocal = new Vector3(4f, 1f, -2f) },
            };

            MassProperties havok = HavokFromBoxes(boxes);
            MassProperties analytic = MassProperties.AnalyticFromBoxes(boxes);

            Assert.That(havok.TotalMass, Is.EqualTo(analytic.TotalMass).Within(1e-2));
            AssertVector3(havok.CenterOfMass, analytic.CenterOfMass, 1e-2,
                "Asymmetric COM must match the analytic weighted mean.");
            AssertInertia(havok.InertiaTensor, analytic.InertiaTensor, 1.5e-2,
                "Asymmetric inertia (including off-diagonals) must match the analytic combination.");
        }

        [Test]
        public void SpawnedShip_LoadsTemplateMassOntoRigidBody()
        {
            var template = new ShipTemplate(
                "hull", 2.5f,
                new[]
                {
                    new TemplateBlock("core", new Vector3I(0, 0, 0), 1500f, 2.5f),
                    new TemplateBlock("nose", new Vector3I(2, 0, 0), 600f, 2.5f),
                });

            SimShip ship = Fc.Spawn(template, SpawnState.At(new Vector3D(10, 20, 30)));

            Assert.That(ship.Id, Is.EqualTo(ship.Grid.EntityId));
            Assert.That(ship.Grid.Body.Mass, Is.EqualTo(2100f).Within(1e-2),
                "The rigid body must carry the template's total dry mass.");
            // core (1500 kg) at cell (0,0,0), nose (600 kg) at cell (2,0,0):
            // COM_x = 600*5 / 2100 = 1.42857 m. The template's blocks must
            // be loaded onto the body so Havok places the COM there.
            Assert.That(Math.Abs(ship.Grid.Body.CenterOfMassLocal.X - 1.428571f), Is.LessThan(1e-2),
                "The body's COM must follow the template mass distribution.");
            Vector3D p = ship.Grid.Body.PositionWorld;
            Assert.That(p.X, Is.EqualTo(10).Within(1e-3));
            Assert.That(p.Y, Is.EqualTo(20).Within(1e-3));
            Assert.That(p.Z, Is.EqualTo(30).Within(1e-3),
                "The body must spawn at the requested world position.");
        }

        [Test]
        public void ZeroGravity_BodyStaysPut()
        {
            // Default world has zero natural gravity; a spawned body at rest
            // must remain exactly where it started.
            SimShip ship = Fc.Spawn(
                new ShipTemplate("hull", 2.5f, new[] { new TemplateBlock("b", Vector3I.Zero, 1000f, 2.5f) }),
                SpawnState.At(new Vector3D(5, 5, 5)));

            Vector3D start = ship.Body.PositionWorld;
            Fc.RunFor(1.0);

            Assert.That(ship.Body.LinearVelocity.LengthSquared(), Is.LessThan(1e-6),
                "Zero gravity must not accelerate a body.");
            Assert.That(Vector3D.Distance(ship.Body.PositionWorld, start), Is.LessThan(1e-3),
                "Zero gravity must not move a body.");
        }

        [Test]
        public void UniformGravity_AcceleratesBody_Downward()
        {
            Fc.World.SetGravity(SimGravityField.Uniform(new Vector3(0, -9.81f, 0)));
            SimShip ship = Fc.Spawn(
                new ShipTemplate("hull", 2.5f, new[] { new TemplateBlock("b", Vector3I.Zero, 1000f, 2.5f) }),
                SpawnState.At(Vector3D.Zero));

            Fc.RunFor(1.0); // 60 fixed steps

            Vector3 v = (Vector3)ship.Body.LinearVelocity;
            Vector3D p = ship.Body.PositionWorld;

            Assert.That(v.Y, Is.EqualTo(-9.81).Within(0.3),
                "After 1 s a free body in uniform gravity must reach ~9.81 m/s downward.");
            Assert.That(v.X, Is.EqualTo(0).Within(1e-3));
            Assert.That(v.Z, Is.EqualTo(0).Within(1e-3));
            Assert.That(p.Y, Is.GreaterThan(-5.5).And.LessThan(-4.5),
                "Gravity must move the body downward by ~0.5·g·t² over 1 s.");
            Assert.That(Math.Abs(p.X), Is.LessThan(1e-3));
            Assert.That(Math.Abs(p.Z), Is.LessThan(1e-3));
        }

        [Test]
        public void ManySpawnsAndDisposals_LeaveCleanWorld()
        {
            // Guards Havok shape/body reference discipline: repeated spawn +
            // dispose must not double-free native objects or leak them across
            // worlds (a crash here would indicate a reference bug).
            for (int round = 0; round < 3; round++)
            {
                var context = new FleetTestContext(new SimWorld());
                try
                {
                    var template = new ShipTemplate(
                        "hull", 2.5f,
                        new[]
                        {
                            new TemplateBlock("a", new Vector3I(0, 0, 0), 100f, 2.5f),
                            new TemplateBlock("b", new Vector3I(1, 0, 0), 100f, 2.5f),
                        });
                    for (int i = 0; i < 5; i++)
                    {
                        context.Spawn(template, SpawnState.At(new Vector3D(i * 10, 0, 0)));
                    }
                    context.RunFor(2.0);
                }
                finally
                {
                    context.Dispose();
                }
            }

            // After the rounds, Havok is still initialized and a fresh world
            // operates cleanly.
            Assert.That(HkBaseSystem.IsInitialized, Is.True);
            using (var world = new SimWorld())
            {
                world.Step();
                Assert.That(world.IsDisposed, Is.False);
            }
        }

        private static MassProperties HavokFromBoxes(IEnumerable<MassProperties.MassElement> boxes)
        {
            var elements = new List<HkMassElement>();
            foreach (MassProperties.MassElement box in boxes)
            {
                elements.Add(new HkMassElement
                {
                    Properties = HkInertiaTensorComputer.ComputeBoxVolumeMassProperties(box.HalfExtents, box.Mass),
                    Tranform = Matrix.CreateTranslation(box.CenterLocal),
                });
            }
            return MassProperties.FromMassElements(elements);
        }

        private static void AssertVector3(Vector3 actual, Vector3 expected, double tolerance, string message)
        {
            Assert.Multiple((Action)(() =>
            {
                Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance), message + " (X)");
                Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance), message + " (Y)");
                Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance), message + " (Z)");
            }));
        }

        private static void AssertInertia(Matrix actual, Matrix expected, double tolerance, string message)
        {
            double[,] a =
            {
                { actual.M11, actual.M12, actual.M13 },
                { actual.M21, actual.M22, actual.M23 },
                { actual.M31, actual.M32, actual.M33 },
            };
            double[,] e =
            {
                { expected.M11, expected.M12, expected.M13 },
                { expected.M21, expected.M22, expected.M23 },
                { expected.M31, expected.M32, expected.M33 },
            };
            Assert.Multiple((Action)(() =>
            {
                for (int row = 0; row < 3; row++)
                {
                    for (int col = 0; col < 3; col++)
                    {
                        double abs = Math.Abs(e[row, col]);
                        double lim = tolerance * (abs > 1e-6 ? abs : 1.0);
                        Assert.That(a[row, col], Is.EqualTo(e[row, col]).Within(lim),
                            string.Format("{0} (element {1}{2})", message, row + 1, col + 1));
                    }
                }
            }));
        }
    }
}
