using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// Deterministic simulation clock: advances in fixed 1/60 s steps, time derived purely
    /// from the tick count, never from wall-clock. Update cadence derives from the same
    /// monotonic counter: Update10 on <c>Tick % 10 == 0</c>, Update100 on <c>Tick % 100 == 0</c>.
    /// </summary>
    public sealed class TickClock
    {
        /// <summary>The fixed physics interval, exactly one sixtieth of a second.</summary>
        public const double FixedStep = 1.0 / 60.0;

        /// <summary>The current monotonic tick. Starts at zero; never goes backwards.</summary>
        public long Tick { get; private set; }

        /// <summary>Simulation time in seconds, an exact multiple of
        /// <see cref="FixedStep"/>.</summary>
        public double Time => Tick * FixedStep;

        /// <summary>Elapsed interval in seconds for <paramref name="steps"/> fixed steps; derived
        /// from step count, not wall-clock.</summary>
        public double TimeSinceLastRun(long steps) => steps * FixedStep;

        /// <summary>Advances the clock by one fixed step.</summary>
        public void Advance()
        {
            Tick++;
        }

        /// <summary>Advances the clock by <paramref name="steps"/> fixed steps.</summary>
        public void Advance(int steps)
        {
            Tick += steps;
        }

        /// <summary>True when <paramref name="tick"/> is an Update10 cadence tick.</summary>
        public bool IsUpdate10Due(long tick) => tick % 10 == 0;

        /// <summary>True when <paramref name="tick"/> is an Update100 cadence tick.</summary>
        public bool IsUpdate100Due(long tick) => tick % 100 == 0;

        /// <summary>Whether a callback at <paramref name="frequency"/> is due on
        /// <paramref name="tick"/> (Update1 every tick; others on cadence).</summary>
        public bool IsUpdateDue(UpdateFrequency frequency, long tick)
        {
            if (frequency == UpdateFrequency.None)
            {
                return false;
            }
            if ((frequency & UpdateFrequency.Update1) != 0)
            {
                return true;
            }
            if ((frequency & UpdateFrequency.Update10) != 0 && tick % 10 == 0)
            {
                return true;
            }
            if ((frequency & UpdateFrequency.Update100) != 0 && tick % 100 == 0)
            {
                return true;
            }
            return false;
        }
    }
}
