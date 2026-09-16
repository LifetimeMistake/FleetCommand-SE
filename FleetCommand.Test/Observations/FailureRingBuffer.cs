using System;
using System.Text;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// The short failure ring buffer (: "Nothing is recorded by
    /// default except a short failure ring buffer"). It is the framework's
    /// one always-on recorder: each tick it retains a short rolling window
    /// of the recent actuator/state history, and an uncaught component
    /// exception is added to it. The fixture surfaces its contents (recent
    /// history) in NUnit output when a test fails. Bounded by a fixed
    /// capacity — oldest entries drop when full.
    /// </summary>
    public sealed class FailureRingBuffer
    {
        private readonly string[] _buffer;
        private int _tail;
        private int _count;

        /// <summary>The fixed capacity in entries.</summary>
        public int Capacity { get; }

        /// <summary>Number of retained entries (&lt;= <see cref="Capacity"/>).</summary>
        public int Count => _count;

        /// <summary>True while the buffer holds no entries.</summary>
        public bool IsEmpty => _count == 0;

        public FailureRingBuffer(int capacity)
        {
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "A failure ring buffer needs a positive capacity.");
            }
            Capacity = capacity;
            _buffer = new string[capacity];
        }

        /// <summary>Adds a line; when full the oldest line drops (ring).</summary>
        public void Add(string line)
        {
            _buffer[_tail] = line ?? string.Empty;
            _tail = (_tail + 1) % Capacity;
            if (_count < Capacity)
            {
                _count++;
            }
        }

        /// <summary>The line at <paramref name="index"/>, oldest first.</summary>
        public string this[int index]
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

        /// <summary>A copy of the retained lines, oldest first.</summary>
        public string[] ToArray()
        {
            string[] result = new string[_count];
            for (int i = 0; i < _count; i++)
            {
                result[i] = this[i];
            }
            return result;
        }

        /// <summary>Renders the retained lines, oldest first, one per line.</summary>
        public override string ToString()
        {
            var sb = new StringBuilder();
            for (int i = 0; i < _count; i++)
            {
                sb.AppendLine(this[i]);
            }
            return sb.ToString();
        }
    }
}
