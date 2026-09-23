using System;
using System.Collections.Generic;
using FleetCommand.Common;
using FleetCommand.Test.Framework;

namespace FleetCommand.Test.Observations
{
    /// <summary>A sampler the world invokes each tick in phase 9 so time-series values can be
    /// captured.</summary>
    internal interface ITickSampler
    {
        /// <summary>Samples the current completed state (tick phase 9).</summary>
        void Sample();
    }

    /// <summary>
    /// A bounded time-series recorder (opt-in via <see cref="FleetTestContext.Record"/>): one
    /// sample per tick from request onward, dropping the oldest when full (ring behaviour).
    /// Numeric accessors (<see cref="MaximumOverLast"/>, <see cref="MinimumOverLast"/>,
    /// <see cref="AverageOverLast"/>) let tests assert on windows without importing samples.
    /// Backed by a <see cref="RingBuffer{T}"/>.
    /// </summary>
    public sealed class TraceRecorder<T> : ITickSampler
    {
        private readonly RingBuffer<T> _ring;
        private readonly Func<FleetTestContext, T> _sample;
        private readonly FleetTestContext _context;

        /// <summary>The fixed capacity; never exceeds this number of samples.</summary>
        public int Capacity { get; }

        /// <summary>The number of samples recorded so far (&lt;= <see cref="Capacity"/>).</summary>
        public int Count => _ring.Count;

        /// <summary>True once <see cref="Count"/> has reached <see cref="Capacity"/>.</summary>
        public bool IsFull => _ring.Count == Capacity;

        /// <summary>The most recent sample recorded (empty when nothing yet).</summary>
        public bool IsEmpty => _ring.Count == 0;

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
            _ring = new RingBuffer<T>(capacity);
        }

        void ITickSampler.Sample()
        {
            Add(_sample(_context));
        }

        /// <summary>Records a sample; when full the oldest sample is dropped (ring).</summary>
        public void Add(T value)
        {
            _ring.Push(value);
        }

        /// <summary>The sample at <paramref name="index"/>, oldest first (index 0 = earliest
        /// retained).</summary>
        public T this[int index] => _ring[index];

        /// <summary>The most recent sample.</summary>
        public T Last()
        {
            if (_ring.Count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no latest sample.");
            }
            return _ring.Last();
        }

        /// <summary>The maximum of the most recent <paramref name="n"/> samples (or all if
        /// fewer).</summary>
        public T MaximumOverLast(int n)
        {
            return ExtremeOverLast(n, maximum: true);
        }

        /// <summary>The minimum of the most recent <paramref name="n"/> samples (or all if
        /// fewer).</summary>
        public T MinimumOverLast(int n)
        {
            return ExtremeOverLast(n, maximum: false);
        }

        private T ExtremeOverLast(int n, bool maximum)
        {
            if (_ring.Count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no window to scan.");
            }
            if (n <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(n));
            }
            int start = _ring.Count - Math.Min(n, _ring.Count);
            T best = this[start];
            for (int i = start + 1; i < _ring.Count; i++)
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

        /// <summary>The arithmetic mean of the most recent <paramref name="n"/> samples (numeric
        /// <c>T</c> only).</summary>
        public double AverageOverLast(int n)
        {
            if (_ring.Count == 0)
            {
                throw new InvalidOperationException("The recorder is empty; there is no window to average.");
            }
            if (n <= 0)
            {
                throw new ArgumentOutOfRangeException(nameof(n));
            }
            int start = _ring.Count - Math.Min(n, _ring.Count);
            double sum = 0.0;
            for (int i = start; i < _ring.Count; i++)
            {
                sum += Convert.ToDouble(this[i]);
            }
            return sum / (_ring.Count - start);
        }

        /// <summary>A copy of the retained samples, oldest first.</summary>
        public T[] ToArray() => _ring.ToArray();

        /// <summary>Discards all retained samples.</summary>
        public void Clear() => _ring.Clear();
    }
}
