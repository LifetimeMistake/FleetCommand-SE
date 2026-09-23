using System;
using System.Text;
using FleetCommand.Common;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// The short failure ring buffer (: "Nothing is recorded by
    /// default except a short failure ring buffer"). It is the framework's
    /// one always-on recorder: each tick it retains a short rolling window
    /// of the recent actuator/state history, and an uncaught component
    /// exception is added to it. The fixture surfaces its contents (recent
    /// history) in NUnit output when a test fails. Bounded by a fixed
    /// capacity — oldest entries drop when full. Storage is a
    /// <see cref="RingBuffer{T}"/>.
    /// </summary>
    public sealed class FailureRingBuffer
    {
        private readonly RingBuffer<string> _ring;

        /// <summary>The fixed capacity in entries.</summary>
        public int Capacity { get; }

        /// <summary>Number of retained entries (&lt;= <see cref="Capacity"/>).</summary>
        public int Count => _ring.Count;

        /// <summary>True while the buffer holds no entries.</summary>
        public bool IsEmpty => _ring.Count == 0;

        public FailureRingBuffer(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "A failure ring buffer needs a positive capacity.");
            }
            Capacity = capacity;
            _ring = new RingBuffer<string>(capacity);
        }

        /// <summary>Adds a line; when full the oldest line drops (ring).</summary>
        public void Add(string line)
        {
            _ring.Push(line ?? string.Empty);
        }

        /// <summary>The line at <paramref name="index"/>, oldest first.</summary>
        public string this[int index] => _ring[index];

        /// <summary>A copy of the retained lines, oldest first.</summary>
        public string[] ToArray() => _ring.ToArray();

        /// <summary>Renders the retained lines, oldest first, one per line.</summary>
        public override string ToString()
        {
            var sb = new StringBuilder();
            for (int i = 0; i < _ring.Count; i++)
            {
                sb.AppendLine(_ring[i]);
            }
            return sb.ToString();
        }
    }
}
