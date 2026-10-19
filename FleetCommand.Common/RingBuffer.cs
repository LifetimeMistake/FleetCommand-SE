using System;
using System.Collections;
using System.Collections.Generic;

namespace FleetCommand.Common
{
    public class RingBuffer<T> : IEnumerable<T>
    {
        private readonly T[] _buffer;
        private int _head;
        private int _tail;
        private int _count;

        public int Count => _count;
        public int Capacity => _buffer.Length;

        public RingBuffer(int capacity)
        {
            if (capacity <= 0)
                throw new InvalidOperationException("Capacity must be greater than zero");
            _buffer = new T[capacity];
            _head = 0;
            _tail = 0;
            _count = 0;
        }

        public void Push(T item)
        {
            _buffer[_head] = item;
            _head = (_head + 1) % _buffer.Length;
            if (_count < _buffer.Length)
                _count++;
            else
                _tail = (_tail + 1) % _buffer.Length;
        }

        public T Pop()
        {
            if (_count == 0)
                throw new InvalidOperationException("Buffer is empty");
            var item = _buffer[_tail];
            _tail = (_tail + 1) % _buffer.Length;
            _count--;
            return item;
        }

        public bool TryPop(out T item)
        {
            if (_count == 0)
            {
                item = default(T);
                return false;
            }
            item = _buffer[_tail];
            _tail = (_tail + 1) % _buffer.Length;
            _count--;
            return true;
        }

        public T Peek()
        {
            if (_count == 0)
                throw new InvalidOperationException("Buffer is empty");
            return _buffer[_tail];
        }

        public bool TryPeek(out T item)
        {
            if (_count == 0)
            {
                item = default(T);
                return false;
            }
            item = _buffer[_tail];
            return true;
        }

        public void Clear()
        {
            Array.Clear(_buffer, 0, _buffer.Length);
            _head = 0;
            _tail = 0;
            _count = 0;
        }

        /// <summary>The item at <paramref name="index"/>, oldest first (index 0 = earliest retained).</summary>
        public T this[int index]
        {
            get
            {
                if (index < 0 || index >= _count)
                {
                    throw new ArgumentOutOfRangeException(nameof(index));
                }
                return _buffer[(_tail + index) % _buffer.Length];
            }
        }

        /// <summary>A copy of the retained items, oldest first.</summary>
        public T[] ToArray()
        {
            T[] result = new T[_count];
            for (int i = 0; i < _count; i++)
            {
                result[i] = _buffer[(_tail + i) % _buffer.Length];
            }
            return result;
        }

        /// <summary>The most recent item (Thrown when the buffer is empty).</summary>
        public T Last()
        {
            if (_count == 0)
            {
                throw new InvalidOperationException("The buffer is empty.");
            }
            return _buffer[(_tail + _count - 1) % _buffer.Length];
        }

        public IEnumerator<T> GetEnumerator()
        {
            if (_count == 0)
                yield break;
            for (int i = 0; i < _count; i++)
                yield return _buffer[(_tail + i) % _buffer.Length];
        }

        IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
    }
}