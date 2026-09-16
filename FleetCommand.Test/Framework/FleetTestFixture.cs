using System;
using NUnit.Framework;
using NUnit.Framework.Interfaces;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Standard test fixture (). Each test gets a fresh, isolated
    /// <see cref="FleetTestContext"/> (with its own world and clock), created
    /// in <see cref="CreateWorld"/> and fully disposed in
    /// <see cref="DestroyWorld"/>, so no physics objects outlive a test even
    /// when it fails. On failure the fixture surfaces the final state and
    /// recent actuator history (: the short failure ring buffer)
    /// in NUnit output.
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
            if (Fc == null)
            {
                return;
            }
            try
            {
                if (TestContext.CurrentContext.Result.Outcome.Status == TestStatus.Failed)
                {
                    WriteFailureDiagnostics(Fc);
                }
            }
            catch
            {
                // Diagnostics must never break teardown or mask the failure.
            }
            Fc.Dispose();
            Fc = null;
        }

        /// <summary>
        /// Writes final state + recent actuator history () to NUnit
        /// output when the test failed, so a failure carries the scenario's
        /// final state and the last ticks of actuator behaviour without
        /// recording anything in an ordinary passing run.
        /// </summary>
        private static void WriteFailureDiagnostics(FleetTestContext fc)
        {
            var world = fc.World;
            TestContext.Out.WriteLine("== FleetTest failure diagnostic (tick " + fc.Tick + ", t=" + fc.Time.ToString("R") + "s) ==");
            TestContext.Out.WriteLine("-- final grid state --");
            for (int i = 0; i < world.Grids.Count; i++)
            {
                var grid = world.Grids[i];
                if (grid.IsDisposed)
                {
                    TestContext.Out.WriteLine("grid " + grid.EntityId + " disposed");
                    continue;
                }
                TestContext.Out.WriteLine(
                    "grid " + grid.EntityId + " pos=" + grid.Body.PositionWorld.ToString()
                    + " vel=" + grid.Body.LinearVelocity.ToString()
                    + " angvel=" + grid.Body.AngularVelocity.ToString());
            }
            TestContext.Out.WriteLine("-- recent actuator history (failure ring buffer) --");
            if (world.FailureLog.IsEmpty)
            {
                TestContext.Out.WriteLine("(empty)");
            }
            else
            {
                TestContext.Out.WriteLine(world.FailureLog.ToString());
            }
            TestContext.Out.WriteLine("-- recent callback cadence --");
            if (world.UpdateCadence.IsEmpty)
            {
                TestContext.Out.WriteLine("(none)");
            }
            else
            {
                var invocations = world.UpdateCadence.ToArray();
                for (int i = 0; i < invocations.Length; i++)
                {
                    TestContext.Out.WriteLine(invocations[i].ToLine());
                }
            }
        }
    }
}
