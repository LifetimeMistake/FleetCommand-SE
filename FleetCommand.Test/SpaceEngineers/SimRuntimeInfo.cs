using System;
using FleetCommand.Test.Framework;
using Sandbox.ModAPI.Ingame;

namespace FleetCommand.Test.SpaceEngineers
{
    /// <summary>
    /// Runtime info for a programmable block's program (, spec
    /// ), implementing the real <see cref="IMyGridProgramRuntimeInfo"/>
    /// (from Sandbox.ModAPI.Ingame). Elapsed simulation time and lifetime
    /// ticks come from the world's fixed tick clock — never wall-clock —
    /// so every value is deterministic. <see cref="TimeSinceLastRun"/> is
    /// derived from the fixed steps since the program's Main last ran (the
    /// same interval a test observes through
    /// <see cref="SimUpdateContext.TimeSinceLastRun"/>). The host supports
    /// the update frequency (settable), the instruction limits it applies,
    /// lifetime ticks, and elapsed-since-last-run; the per-run
    /// instruction counters are not instrumented by the simulator and
    /// throw a naming <see cref="NotSupportedException"/>.
    /// </summary>
    public sealed class SimRuntimeInfo : IMyGridProgramRuntimeInfo
    {
        // The programmable-block runtime limits the host applies (CLAUDE.md:
        // ~50,000 significant instructions, ~10,000 nested method calls per run).
        public const int InstructionLimit = 50000;
        public const int CallChainLimit = 10000;

        private readonly SimWorld _world;
        private readonly long _startTick;
        private long _lastRunTick;
        private bool _hasRun;

        internal SimRuntimeInfo(SimWorld world)
        {
            _world = world ?? throw new ArgumentNullException(nameof(world));
            _startTick = world.Clock.Tick;
        }

        /// <summary>Gets or sets how frequently the program will run itself.</summary>
        public UpdateFrequency UpdateFrequency { get; set; }

        /// <summary>
        /// The number of fixed simulation ticks since this program was
        /// started (construction), the deterministic stand-in for the
        /// game's "ticks running, reset on recompile".
        /// </summary>
        public long LifetimeTicks => _world.Clock.Tick - _startTick;

        /// <summary>
        /// The elapsed simulation time since the program's Main last ran,
        /// derived from the fixed tick clock. Returns
        /// <see cref="TimeSpan.Zero"/> before the first run (the game's
        /// "no valid data in the constructor").
        /// </summary>
        public TimeSpan TimeSinceLastRun
        {
            get
            {
                if (!_hasRun)
                {
                    return TimeSpan.Zero;
                }
                long elapsedTicks = _world.Clock.Tick - _lastRunTick;
                return TimeSpan.FromSeconds(TickClock.FixedStep * elapsedTicks);
            }
        }

        /// <summary>The maximum significant instructions per run the host allows.</summary>
        public int MaxInstructionCount => InstructionLimit;

        /// <summary>The maximum nested method calls the host allows.</summary>
        public int MaxCallChainDepth => CallChainLimit;

        /// <summary>
        /// Records that the program's Main ran at the current tick so
        /// <see cref="TimeSinceLastRun"/> reflects the actual elapsed
        /// interval. Called by <see cref="SimProgrammableBlock.TryRun"/>;
        /// also an explicit hook for tests that drive a program directly
        /// ().
        /// </summary>
        public void MarkRun()
        {
            _lastRunTick = _world.Clock.Tick;
            _hasRun = true;
        }

        // — counters the host does not instrument —

        /// <inheritdoc />
        public double LastRunTimeMs => ThrowNotSupported<double>("IMyGridProgramRuntimeInfo.LastRunTimeMs");

        /// <inheritdoc />
        public int CurrentInstructionCount => ThrowNotSupported<int>("IMyGridProgramRuntimeInfo.CurrentInstructionCount");

        /// <inheritdoc />
        public int CurrentCallChainDepth => ThrowNotSupported<int>("IMyGridProgramRuntimeInfo.CurrentCallChainDepth");

        /// <summary>Throws the canonical not-implemented exception naming <paramref name="member"/>.</summary>
        private static TResult ThrowNotSupported<TResult>(string member)
        {
            throw new NotSupportedException(
                "SimRuntimeInfo does not implement '" + member + "' (outside the  contract, ).");
        }
    }
}
