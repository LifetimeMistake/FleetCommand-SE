using System;
using System.Threading;
using FleetCommand.Test.Framework;
using FleetCommand.Test.Harness;
using Havok;
using VRage.Library.Threading;

namespace FleetCommand.Test.Physics
{
    /// <summary>
    /// Owns the process-wide Havok lifetime. Mirrors how the game boots and
    /// shuts down Havok (<c>MySandboxGame.LoadData_UpdateThread</c> /
    /// <c>UnloadData_UpdateThread</c>): <see cref="HkBaseSystem.Init"/> once per
    /// process, <see cref="HkBaseSystem.Quit"/> once per process, each wrapped
    /// in <see cref="HkAccessControl.AccessState.Exclusive"/>. All calls are
    /// idempotent and paired.
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
        /// Initializes Havok exactly once per process. No-op on repeat calls.
        /// The critical section argument is required — Havok dereferences it
        /// during <see cref="HkBaseSystem.Quit"/> (a <c>null</c> causes a
        /// <c>NullReferenceException</c> in <c>HkShape.ReportOwnership</c>).
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
                    // Positional args (C# 6): memory size 16 MiB, log writer,
                    // deep profiling off, shared critical section — mirrors
                    // MySandboxGame.LoadData().
                    HkBaseSystem.Init(16 * 1024 * 1024, s_logWriter, false, SharedSection);
                }

                s_profile = SePhysicsProfile.Default;
                s_initialized = true;
            }
        }

        /// <summary>
        /// Shuts down Havok exactly once per process. No-op on repeat calls so
        /// it is safe under repeated fixture teardown.
        /// </summary>
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

        /// <summary>
        /// A minimal <see cref="ISharedCriticalSection"/> implementation. The
        /// game normally uses a mutex here; a monitor is equivalent for the
        /// single-threaded, deterministic test host.
        /// </summary>
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
