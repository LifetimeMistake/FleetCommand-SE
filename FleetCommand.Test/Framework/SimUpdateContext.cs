using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Context handed to per-Update1 callbacks registered with
    /// <see cref="FleetTestContext.EveryUpdate1"/>. Exposes coherent
    /// start-of-tick simulation state — time, the elapsed interval since the
    /// last callback, and the update source — and never supplies future state
    /// (). The owning ship/program fields are filled in by the later
    /// stages that introduce those blocks.
    /// </summary>
    public sealed class SimUpdateContext
    {
        /// <summary>
        /// Simulation time, in seconds, at the start of the current tick.
        /// An exact multiple of 1/60 s.
        /// </summary>
        public double Time { get; }

        /// <summary>
        /// The elapsed simulation interval, in seconds, since the previous
        /// Update1 callback for this callback. Derived from the count of
        /// elapsed fixed steps, never from wall-clock time. When an Update1
        /// was missed on an earlier tick, this carries the accumulated
        /// interval so the consumer sees real simulation time advance.
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
