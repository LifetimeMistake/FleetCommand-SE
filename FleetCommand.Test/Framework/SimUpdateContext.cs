using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Context handed to per-Update1 callbacks registered with
    /// <see cref="FleetTestContext.EveryUpdate1"/>: coherent start-of-tick time, the
    /// elapsed interval since the last callback, and the update source. Never future state.
    /// </summary>
    public sealed class SimUpdateContext
    {
        /// <summary>Simulation time (s) at the start of the current tick; an exact multiple of 1/60 s.</summary>
        public double Time { get; }

        /// <summary>
        /// Elapsed simulation interval (s) since the previous Update1 callback, from the count
        /// of elapsed fixed steps, never wall-clock. Carries accumulated time across a missed tick.
        /// </summary>
        public double TimeSinceLastRun { get; }

        /// <summary>The update source that drove this invocation.</summary>
        public UpdateType UpdateSource { get; }

        internal SimUpdateContext(double time, double timeSinceLastRun, UpdateType updateSource)
        {
            Time = time;
            TimeSinceLastRun = timeSinceLastRun;
            UpdateSource = updateSource;
        }
    }
}
