using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using Havok;
using NUnit.Framework;
using VRageMath;

namespace FleetCommand.Test.Tests.FrameworkTests
{
    /// <summary>
    /// Verifies world ownership and disposal (, ): each world
    /// owns its Havok objects and disposes them fully, and a complete
    /// disposal is followed by a clean second world in the same process —
    /// with Havok remaining initialized across both (init once, quit once per
    /// process).
    /// </summary>
    [TestFixture]
    public class WorldOwnershipTests
    {
        [Test]
        public void Dispose_ReleasesTheHkWorld()
        {
            var world = new SimWorld();
            Assert.That(world.IsDisposed, Is.False);
            world.Dispose();
            Assert.That(world.IsDisposed, Is.True, "World must report disposed after Dispose.");
        }

        [Test]
        public void Dispose_IsIdempotent()
        {
            var world = new SimWorld();
            world.Dispose();
            world.Dispose(); // must not throw
            Assert.That(world.IsDisposed, Is.True);
        }

        [Test]
        public void Step_AfterDispose_Throws()
        {
            var world = new SimWorld();
            world.Dispose();
            Assert.Throws<ObjectDisposedException>((Action)(() => world.Step()));
        }

        [Test]
        public void CompleteDisposal_CleanSecondWorld()
        {
            // Two isolated worlds in one process: the first is fully
            // disposed, then the second is created and operates cleanly.
            // Havok remains initialized across both (init once, quit once).
            RunOneWorld();
            RunOneWorld();

            Assert.That(HkBaseSystem.IsInitialized, Is.True,
                "Havok must remain initialized across both worlds.");
        }

        private static void RunOneWorld()
        {
            var world = new SimWorld();
            try
            {
                for (int i = 0; i < 20; i++)
                {
                    world.Step();
                }
                Assert.That(world.IsDisposed, Is.False);
            }
            finally
            {
                world.Dispose();
            }
            Assert.That(world.IsDisposed, Is.True,
                "The first world must be fully disposed before the second is created.");
        }

        [Test]
        public void ContextDispose_DisposesWorld()
        {
            var context = new FleetTestContext(new SimWorld());
            Assert.That(context.World.IsDisposed, Is.False);
            context.Dispose();
            Assert.That(context.World.IsDisposed, Is.True,
                "Disposing the context must dispose its world.");
        }

        [Test]
        public void Spawn_CreatesShip_WithBody_InWorld()
        {
            using (var context = new FleetTestContext(new SimWorld()))
            {
                var template = SingleBlockTemplate("test", 5000);
                var ship = context.Spawn(template, SpawnState.At(new Vector3D(10, 0, 0)));

                Assert.That(ship, Is.Not.Null);
                Assert.That(context.World.Bodies, Has.Count.EqualTo(1));
                Assert.That(ship.Grid.Body, Is.Not.Null);
                Assert.That(ship.Grid.Body.Mass, Is.EqualTo(5000f).Within(1e-3));
                Vector3D p = ship.Grid.Body.PositionWorld;
                Assert.That(p.X, Is.EqualTo(10).Within(1e-3));
                Assert.That(p.Y, Is.EqualTo(0).Within(1e-3));
                Assert.That(p.Z, Is.EqualTo(0).Within(1e-3));
            }
        }

        [Test]
        public void Dispose_DisposesSpawnedBodies()
        {
            var context = new FleetTestContext(new SimWorld());
            var template = SingleBlockTemplate("test", 5000);
            context.Spawn(template, SpawnState.At(Vector3D.Zero));
            context.Spawn(template, SpawnState.At(new Vector3D(5, 0, 0)));
            Assert.That(context.World.Bodies, Has.Count.EqualTo(2));

            context.Dispose();

            Assert.That(context.World.IsDisposed, Is.True,
                "Disposing the context must dispose its world and all spawned bodies.");
        }

        private static FleetCommand.Test.Templates.ShipTemplate SingleBlockTemplate(string name, float mass)
        {
            return new FleetCommand.Test.Templates.ShipTemplate(
                name, 2.5f,
                new[]
                {
                    new FleetCommand.Test.Templates.TemplateBlock(
                        name + "-block", new VRageMath.Vector3I(0, 0, 0), mass, 2.5f),
                });
        }

        [Test]
        public void Record_ReturnsABoundedRecorder()
        {
            using (var context = new FleetTestContext(new SimWorld()))
            {
                FleetCommand.Test.Observations.TraceRecorder<double> recorder = context.Record(c => c.Time, 4);
                context.Step(6);
                Assert.That(recorder.Count, Is.EqualTo(4),
                    "Record returns a bounded time-series recorder ( observations).");
            }
        }
    }
}
