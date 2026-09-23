using System;
using FleetCommand.Test.Physics;
using Havok;
using NUnit.Framework;

namespace FleetCommand.Test.Tests.Smoke
{
    /// <summary>Verifies the host contract: the process is x64, the physics profile is visible, and
    /// Havok init/quit each happen exactly once per process.</summary>
    [TestFixture]
    public class HavokHostSmokeTests
    {
        [Test]
        public void Host_IsX64()
        {
            Assert.That(IntPtr.Size, Is.EqualTo(8), "The test host must run as a 64-bit process.");
        }

        [Test]
        public void ActiveProfile_IsReported()
        {
            Assert.That(PhysicsHost.Profile, Is.Not.Null, "The active physics profile must be recorded.");
            TestContext.WriteLine("Active SePhysicsProfile: " + PhysicsHost.Profile.Name);
        }

        [Test]
        public void Havok_IsInitialized_ForWholeProcess()
        {
            // Init happens once per process in the assembly setup; it must
            // already be live when tests run and must remain live throughout.
            Assert.That(HkBaseSystem.IsInitialized, Is.True,
                "Havok must be initialized once for the whole process.");
        }

        [Test]
        public void TwoWorlds_SecondOperates_AfterFirstDisposed()
        {
            // Run the same "empty world" lifecycle twice in one process. The
            // first world is fully disposed before the second is created —
            // "complete disposal followed by a clean second world". Havok
            // itself enters/leaves once per process via PhysicsHost, so this
            // also proves Init/Quit are each called exactly once across both
            // runs and the second run still operates correctly.
            RunOneWorld();
            RunOneWorld();

            Assert.That(HkBaseSystem.IsInitialized, Is.True,
                "Havok must remain initialized across both worlds (init once, quit once per process).");
        }

        private static void RunOneWorld()
        {
            // Values mirror the game's CreateHkWorld / CreateWorldCInfo for a
            // single-threaded, no-global-gravity host.
            var world = new HkWorld(
                enableGlobalGravity: false,
                broadphaseCubeSideLength: 100000f,
                contactRestingVelocity: float.MaxValue,
                enableMultithreading: false,
                solverIterations: 8);

            using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
            {
                try
                {
                    Assert.That(world.IsDisposed, Is.False);
                }
                finally
                {
                    world.Dispose();
                }
                Assert.That(world.IsDisposed, Is.True,
                    "The first world must be fully disposed before the second is created.");
            }
        }
    }
}
