using System;

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
    /// </summary>
    public sealed class UpdateCadenceLog
    {
        private readonly UpdateInvocation[] _buffer;
        private int _tail;
        private int _count;

        /// <summary>The fixed capacity.</summary>
        public int Capacity { get; }

        /// <summary>Number of retained invocations.</summary>
        public int Count => _count;

        public bool IsEmpty => _count == 0;

        public UpdateCadenceLog(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "A cadence log needs a positive capacity.");
            }
            Capacity = capacity;
            _buffer = new UpdateInvocation[capacity];
        }

        internal void Add(UpdateInvocation invocation)
        {
            _buffer[_tail] = invocation;
            _tail = (_tail + 1) % Capacity;
            if (_count < Capacity)
            {
                _count++;
            }
        }

        /// <summary>The invocation at <paramref name="index"/>, oldest first.</summary>
        public UpdateInvocation this[int index]
        {
            get
            {
                if (index < 0 || index >= _count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }
                return _buffer[(_tail - _count + index + Capacity) % Capacity];
            }
        }

        /// <summary>A copy of the retained invocations, oldest first.</summary>
        public UpdateInvocation[] ToArray()
        {
            UpdateInvocation[] result = new UpdateInvocation[_count];
            for (int i = 0; i < _count; i++)
            {
                result[i] = this[i];
            }
            return result;
        }

        /// <summary>The most recent invocation.</summary>
        public UpdateInvocation Last()
        {
            if (_count == 0)
            {
                throw new InvalidOperationException("The cadence log is empty.");
            }
            return this[_count - 1];
        }
    }
}
