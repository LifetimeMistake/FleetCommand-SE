using System;
using FleetCommand.Common;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// One Update1 callback invocation. <see cref="SinceLastRun"/> is the interval since
    /// this callback last ran (accumulating over a missed Update1);
    /// <see cref="AppliedInterval"/> is the exact fixed step this invocation's commands
    /// act over — the same tick's physics interval — so observers know the applied interval.
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

    /// <summary>A short rolling log of recent <see cref="UpdateInvocation"/>s, keeping cadence and command-application intervals observable in failure output. Backed by a <see cref="RingBuffer{T}"/>.</summary>
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
