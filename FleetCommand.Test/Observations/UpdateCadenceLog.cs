using System;
using FleetCommand.Common;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// One Update1 callback invocation ( "callback cadence and
    /// exact command-application intervals"). <see cref="SinceLastRun"/>
    /// is the simulation interval since this callback last ran (which
    /// accumulates over a missed Update1); <see cref="AppliedInterval"/>
    /// is the exact fixed step this invocation's commands act over — the
    /// phase 6–7 physics interval of the same tick — so observers can
    /// always know the interval a command was actually applied over
    /// without inferring it from a requested cadence.
    /// </summary>
    public struct UpdateInvocation
    {
        /// <summary>The tick on which the callback ran.</summary>
        public long Tick;

        /// <summary>Simulation time (s) at the start of that tick.</summary>
        public double Time;

        /// <summary>Elapsed simulation time (s) since this callback last ran.</summary>
        public double SinceLastRun;

        /// <summary>The exact fixed step (s) this command was applied over.</summary>
        public double AppliedInterval;

        /// <summary>A one-line summary suitable for diagnostics.</summary>
        public string ToLine()
        {
            return "tick " + Tick + " update1 t=" + Time.ToString("R")
                + " since=" + SinceLastRun.ToString("R")
                + " applied=" + AppliedInterval.ToString("R") + "s";
        }
    }

    /// <summary>
    /// A short rolling log of recent <see cref="UpdateInvocation"/>s, kept by
    /// the world so callback cadence and exact command-application intervals
    /// are observable () and appear in failure output. Bounded.
    /// Storage is a <see cref="RingBuffer{T}"/>.
    /// </summary>
    public sealed class UpdateCadenceLog
    {
        private readonly RingBuffer<UpdateInvocation> _ring;

        /// <summary>The fixed capacity.</summary>
        public int Capacity { get; }

        /// <summary>Number of retained invocations.</summary>
        public int Count => _ring.Count;

        public bool IsEmpty => _ring.Count == 0;

        public UpdateCadenceLog(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "A cadence log needs a positive capacity.");
            }
            Capacity = capacity;
            _ring = new RingBuffer<UpdateInvocation>(capacity);
        }

        internal void Add(UpdateInvocation invocation)
        {
            _ring.Push(invocation);
        }

        /// <summary>The invocation at <paramref name="index"/>, oldest first.</summary>
        public UpdateInvocation this[int index] => _ring[index];

        /// <summary>A copy of the retained invocations, oldest first.</summary>
        public UpdateInvocation[] ToArray() => _ring.ToArray();

        /// <summary>The most recent invocation.</summary>
        public UpdateInvocation Last()
        {
            if (_ring.Count == 0)
            {
                throw new InvalidOperationException("The cadence log is empty.");
            }
            return _ring.Last();
        }
    }
}
