using System;
using System.Collections.Generic;
using FleetCommand.Test.Framework;

namespace FleetCommand.Test.Observations
{
    /// <summary>
    /// A TickSampler the world invokes each tick in phase 9 so time-series
    /// values can be captured. The failure ring buffer is the only always-on
    /// sampler (); a <see cref="TraceRecorder{T}"/> is opt-in via
    /// <see cref="FleetTestContext.Record"/>.
    /// </summary>
    internal interface ITickSampler
    {
        /// <summary>Samples the current completed state (tick phase 9).</summary>
        void Sample();
    }

    /// <summary>
    /// A bounded time-series recorder (: "<c>TraceRecorder&lt;T&gt;</c>
    /// uses a fixed or caller-specified capacity"). Records one sample per
    /// tick from the time it is requested (registered) onward; when full it
    /// drops the oldest sample (ring behaviour), so it is never unbounded.
    /// Nothing is recorded unless the caller asks for a recorder. Numeric
    /// convenience accessors (<see cref="MaximumOverLast"/>,
    /// <see cref="MinimumOverLast"/>, <see cref="AverageOverLast"/>) let tests
    /// assert on recent windows without importing the samples.
    /// </summary>
    public sealed class TraceRecorder<T> : ITickSampler
    {
        private readonly T[] _buffer;
        private int _tail;
        private int _count;
        private readonly Func<FleetTestContext, T> _sample;
        private readonly FleetTestContext _context;

        /// <summary>The fixed capacity; never exceeds this number of samples.</summary>
        public int Capacity { get; }

        /// <summary>The number of samples recorded so far (&lt;= <see cref="Capacity"/>).</summary>
        public int Count => _count;

        /// <summary>True once <see cref="Count"/> has reached <see cref="Capacity"/>.</summary>
        public bool IsFull => _count == Capacity;

        /// <summary>The most recent sample recorded (empty when nothing yet).</summary>
        public bool IsEmpty => _count == 0;

        internal TraceRecorder(Func<FleetTestContext, T> sample, FleetTestContext context, int capacity)
        {
            if (sample == null)
            {
                throw new ArgumentNullException(nameof(sample));
            }
            if (context == null)
            {
                throw new ArgumentNullException(nameof(context));
            }
            if (capacity <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(capacity), "A recorder needs a positive capacity.");
            }
            _sample = sample;
            _context = context;
            Capacity = capacity;
            _buffer = new T[capacity];
        }

        void ITickSampler.Sample()
        {
            Add(_sample(_context));
        }

        /// <summary>Records a sample; when full the oldest sample is dropped (ring).</summary>
        public void Add(T value)
        {
            _buffer[_tail] = value;
            _tail = (_tail + 1) % Capacity;
            if (_count < Capacity)
            {
                _count++;
            }
        }

        /// <summary>The sample at <paramref name="index"/>, oldest first (index 0 = earliest retained).</summary>
        public T this[int index]
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

        /// <summary>The most recent sample.</summary>
        public T Last()
        {
            if (_count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no latest sample.");
            }
            return this[_count - 1];
        }

        /// <summary>The maximum of the most recent <paramref name="n"/> samples (or all if fewer).</summary>
        public T MaximumOverLast(int n)
        {
            return ExtremeOverLast(n, maximum: true);
        }

        /// <summary>The minimum of the most recent <paramref name="n"/> samples (or all if fewer).</summary>
        public T MinimumOverLast(int n)
        {
            return ExtremeOverLast(n, maximum: false);
        }

        private T ExtremeOverLast(int n, bool maximum)
        {
            if (_count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no window to scan.");
            }
            if (n <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(n));
            }
            int start = _count - Math.Min(n, _count);
            T best = this[start];
            for (int i = start + 1; i < _count; i++)
            {
                T candidate = this[i];
                int cmp = Comparer<T>.Default.Compare(candidate, best);
                if ((maximum && cmp > 0) || (!maximum && cmp < 0))
                {
                    best = candidate;
                }
            }
            return best;
        }

        /// <summary>The arithmetic mean of the most recent <paramref name="n"/> samples (numeric <c>T</c> only).</summary>
        public double AverageOverLast(int n)
        {
            if (_count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no window to average.");
            }
            if (n <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(n));
            }
            int start = _count - Math.Min(n, _count);
            double sum = 0.0;
            for (int i = start; i < _count; i++)
            {
                sum += Convert.ToDouble(this[i]);
            }
            return sum / (_count - start);
        }

        /// <summary>A copy of the retained samples, oldest first.</summary>
        public T[] ToArray()
        {
            T[] result = new T[_count];
            for (int i = 0; i < _count; i++)
            {
                result[i] = this[i];
            }
            return result;
        }

        /// <summary>Discards all retained samples.</summary>
        public void Clear()
        {
            _tail = 0;
            _count = 0;
            Array.Clear(_buffer, 0, _buffer.Length);
        }
    }
}
