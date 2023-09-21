using System;
using System.CodeDom;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Networking.Math
{
    public struct Range<T> where T : IEquatable<T>, IComparable<T>
    {
        public T From;
        public T To;

        public Range(T from, T to)
        {
            if (to.CompareTo(from) == -1)
                throw new ArgumentException("Invalid range");

            From = from;
            To = to;
        }

        public bool Equals(Range<T> other)
        {
            return (this.From.Equals(other.From) && this.To.Equals(other.To));
        }

        public bool Intersects(Range<T> other)
        {
            return (this.From.CompareTo(other.To) <= 0 && this.To.CompareTo(other.From) >= 0);
        }

        public bool Contains(T value)
        {
            return (this.From.CompareTo(value) <= 0 && this.To.CompareTo(value) >= 0);
        }
    }
}
