using System;
using System.Threading;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Harness;
using Havok;
using VRage.Library.Threading;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// Owns the process-wide Havok lifetime, mirroring the game's boot/shutdown
    /// (Init/Quit once per process, each under Exclusive — see MySandboxGame).
    /// </summary>
    public static class PhysicsHost
    {
        private static readonly object s_lock = new object();
        private static bool s_initialized;
        private static SePhysicsProfile s_profile;
        private static readonly Action<string> s_logWriter = WriteHavokLog;

        /// <summary>The active compatibility profile, printed in test output.</summary>
        public static SePhysicsProfile Profile
        {
            get
            {
                lock (s_lock)
                {
                    return s_profile;
                }
            }
        }

        /// <summary>
        /// Initializes Havok once per process; no-op on repeat calls. The critical section
        /// arg is required — Havok dereferences it during <see cref="HkBaseSystem.Quit"/>.
        /// </summary>
        public static void Initialize()
        {
            lock (s_lock)
            {
                if (s_initialized)
                {
                    return;
                }

                SeReferenceResolver.EnsureRegistered();

                using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                {
                    // 16 MiB memory, log writer, profiling off, shared critical section
                    // (MySandboxGame.LoadData()).
                    HkBaseSystem.Init(16 * 1024 * 1024, s_logWriter, false, SharedSection);
                }

                s_profile = SePhysicsProfile.Default;
                s_initialized = true;
            }
        }

        /// <summary>Shuts down Havok once per process; no-op on repeat calls (safe under repeated
        /// teardown).</summary>
        public static void Shutdown()
        {
            lock (s_lock)
            {
                if (!s_initialized)
                {
                    return;
                }

                using (HkAccessControl.PushState(HkAccessControl.AccessState.Exclusive))
                {
                    HkBaseSystem.Quit();
                }

                s_initialized = false;
                s_profile = null;
            }
        }

        /// <summary>Minimal <see cref="ISharedCriticalSection"/>; a monitor equals the game's mutex
        /// on the single-threaded test host.</summary>
        private static readonly ISharedCriticalSection SharedSection = new MonitorSharedSection();

        private sealed class MonitorSharedSection : ISharedCriticalSection
        {
            private readonly object _lock = new object();

            public SharedCriticalSection_UniqueLock EnterUnique()
            {
                Monitor.Enter(_lock);
                return new SharedCriticalSection_UniqueLock(this);
            }

            public SharedCriticalSection_SharedLock EnterShared()
            {
                Monitor.Enter(_lock);
                return new SharedCriticalSection_SharedLock(this);
            }

            public void LeaveUnique_Internal()
            {
                Monitor.Exit(_lock);
            }

            public void LeaveShared_Internal()
            {
                Monitor.Exit(_lock);
            }

            public void Dispose()
            {
            }
        }

        private static void WriteHavokLog(string text)
        {
            System.Console.WriteLine("Havok: " + text);
        }
    }
}
