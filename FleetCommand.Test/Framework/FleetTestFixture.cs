using NUnit.Framework;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Standard test fixture (). Each test gets a fresh, isolated
    /// <see cref="FleetTestContext"/> (with its own world and clock), created
    /// in <see cref="CreateWorld"/> and fully disposed in
    /// <see cref="DestroyWorld"/>, so no physics objects outlive a test even
    /// when it fails.
    /// </summary>
    public abstract class FleetTestFixture
    {
        /// <summary>The fixture's scenario context.</summary>
        protected FleetTestContext Fc { get; private set; }

        [SetUp]
        public void CreateWorld()
        {
            Fc = new FleetTestContext(new SimWorld());
        }

        [TearDown]
        public void DestroyWorld()
        {
            if (Fc != null)
            {
                Fc.Dispose();
                Fc = null;
            }
        }
    }
}
