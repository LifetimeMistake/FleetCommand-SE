using System;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Physics;
using Havok;
using NUnit.Framework;

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
        public void Spawn_IsUnsupported_UntilStage3()
        {
            using (var context = new FleetTestContext(new SimWorld()))
            {
                var template = new FleetCommand.Test.Templates.ShipTemplate();
                var state = new SpawnState();
                Assert.Throws<NotSupportedException>((Action)(() => context.Spawn(template, state)),
                    "Spawn must report itself unsupported until /10, naming the member.");
            }
        }

        [Test]
        public void Record_IsUnsupported_UntilStage11()
        {
            using (var context = new FleetTestContext(new SimWorld()))
            {
                Assert.Throws<NotSupportedException>(
                    (Action)(() => context.Record<double>(c => c.Time)),
                    "Record must report itself unsupported until , naming the member.");
            }
        }
    }
}
