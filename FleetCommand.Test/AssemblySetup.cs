using FleetCommand.Test.Physics;
using NUnit.Framework;

namespace FleetCommand.Test
{
    /// <summary>
    /// Assembles the process-wide test host. <see cref="Init"/> boots Havok
    /// exactly once before any test runs; <see cref="Shutdown"/> quits it
    /// exactly once after all tests have run. Because Havok enters/leaves once
    /// per process, this is the single place that owns the lifetime.
    /// </summary>
    [SetUpFixture]
    public sealed class HavokAssemblySetup
    {
        [OneTimeSetUp]
        public void Init()
        {
            PhysicsHost.Initialize();
        }

        [OneTimeTearDown]
        public void Shutdown()
        {
            PhysicsHost.Shutdown();
        }
    }
}
