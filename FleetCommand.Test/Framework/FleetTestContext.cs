using System;
using FleetCommand.Test.Observations;
using FleetCommand.Test.Physics;
using FleetCommand.Test.Templates;

namespace FleetCommand.Test.Framework
{
    /// <summary>
    /// The primary test surface (): owns the isolated world,
    /// deterministic clock, update scheduler, and trace recorders, and
    /// forwards scenario controls to it. One context per test; disposed by the
    /// fixture teardown.
    /// </summary>
    public sealed class FleetTestContext : IDisposable
    {
        private bool _disposed;

        /// <summary>The isolated simulation world.</summary>
        public SimWorld World { get; }

        /// <summary>The current tick (monotonic).</summary>
        public long Tick => World.Clock.Tick;

        /// <summary>Simulation time in seconds (an exact multiple of 1/60).</summary>
        public double Time => World.Clock.Time;

        internal FleetTestContext(SimWorld world)
        {
            World = world;
        }

        /// <summary>
        /// Spawns a ship. Strictly the framework's block surface arrives in
        /// Stages 3 and 10; until <see cref="ShipTemplate"/> and the rigid
        /// grid exist this member reports itself unsupported rather than
        /// silently doing nothing.
        /// </summary>
        public SimShip Spawn(ShipTemplate template, in SpawnState initialState)
        {
            throw new NotSupportedException(
                "FleetTestContext.Spawn is not implemented until  (rigid grid) and  (templates) land.");
        }

        /// <summary>Advances the simulation by one fixed 1/60 s step.</summary>
        public void Step()
        {
            World.Step();
        }

        /// <summary>Advances the simulation by <paramref name="ticks"/> fixed steps.</summary>
        public void Step(int ticks)
        {
            World.Step(ticks);
        }

        /// <summary>Runs for <paramref name="seconds"/> of simulation time.</summary>
        public void RunFor(double seconds)
        {
            World.RunFor(seconds);
        }

        /// <summary>
        /// Steps until <paramref name="condition"/> (evaluated against this
        /// context) is satisfied by a completed-state observation, or
        /// <paramref name="timeoutSeconds"/> elapses. Stops at the first
        /// tick whose completed-state observation satisfies the predicate;
        /// returns false on timeout.
        /// </summary>
        public bool RunUntil(Func<FleetTestContext, bool> condition, double timeoutSeconds)
        {
            return World.RunUntil(() => condition(this), timeoutSeconds);
        }

        /// <summary>Schedules a scenario mutation to run on <paramref name="tick"/>.</summary>
        public void AtTick(long tick, Action<FleetTestContext> action)
        {
            World.AtTick(tick, () => action(this));
        }

        /// <summary>Registers a per-Update1 callback with start-of-tick state.</summary>
        public void EveryUpdate1(Action<SimUpdateContext> action)
        {
            World.EveryUpdate1(action);
        }

        /// <summary>
        /// Requests a bounded time-series recorder. Strictly arrivals in
        /// ; until then this member reports itself unsupported
        /// rather than silently recording nothing.
        /// </summary>
        public TraceRecorder<T> Record<T>(Func<FleetTestContext, T> sample)
        {
            throw new NotSupportedException(
                "FleetTestContext.Record is not implemented until  (observations) lands.");
        }

        public void Dispose()
        {
            if (_disposed)
            {
                return;
            }
            World.Dispose();
            _disposed = true;
        }
    }
}
