using System;

namespace IngameScript.IO
{
    public class MemoryStream : Stream
    {
        private const int BLOCK_SIZE = 128;
        private byte[] _data;
        private int _position;
        private int _length;
        private int _capacity;
        private bool _isOpen;
        private bool _writable;
        private bool _expandable;

        public override int Position
        {
            get
            {
                EnsureNotClosed();
                return _position;
            }

            set
            {
                Seek(value);
            }
        }

        public override int Length
        {
            get
            {
                EnsureNotClosed();
                return _length;
            }
        }

        public override int Capacity
        {
            get
            {
                EnsureNotClosed();
                return _capacity;
            }

            set
            {
                ResizeStream(value);
            }
        }

        public override bool CanSeek
        {
            get
            {
                return _isOpen;
            }
        }

        public override bool CanRead
        {
            get
            {
                return _isOpen;
            }
        }

        public override bool CanWrite
        {
            get
            {
                return _isOpen && _writable;
            }
        }

        public MemoryStream(int capacity)
        {
            if (capacity < 0)
            {
                throw new Exception("Capacity must be greater than 0");
            }

            _data = new byte[0];
            _position = 0;
            _length = 0;
            _capacity = 0;
            _writable = true;
            _expandable = true;
            _isOpen = true;
        }

        public MemoryStream() : this(0)
        { }

        public MemoryStream(byte[] buffer, bool writable)
        {
            _data = buffer;
            _position = 0;
            _length = buffer.Length;
            _capacity = buffer.Length;
            _writable = writable;
            _expandable = false;
            _isOpen = true;
        }

        public MemoryStream(byte[] buffer) : this(buffer, true)
        { }

        private void EnsureNotClosed()
        {
            if (!_isOpen)
                StreamErrors.StreamIsClosed();
        }

        private void ResizeStream(int capacity)
        {
            EnsureNotClosed();

            if (!_expandable)
                throw new Exception("Stream does not support resizing.");

            if (!_writable)
                StreamErrors.WriteNotSupported();

            byte[] data = new byte[capacity];
            Array.Copy(_data, data, Math.Min(_capacity, capacity));
            _data = data;
            _capacity = capacity;
            _length = Math.Min(_length, capacity);
        }

        private void EnsureCapacity(int length)
        {
            if (length < 0)
                throw new Exception("Stream too long");

            if (length > _capacity)
            {
                if (length < BLOCK_SIZE)
                    length = BLOCK_SIZE;

                if (length < _capacity * 2)
                    length = _capacity * 2;

                if (_capacity * 2 > Int32.MaxValue)
                    length = ((length > 2147483591) ? length : 2147483591);

                ResizeStream(length);
            }
        }

        public override void Flush()
        { }

        public override void Dispose(bool disposing)
        {
            try
            {
                if (disposing)
                {
                    _data = null;
                    _isOpen = false;
                    _writable = false;
                }
            }
            finally
            {
                base.Dispose(disposing);
            }
        }

        public override void SetLength(int length)
        {
            if (length < 0 || length > Int32.MaxValue)
                StreamErrors.LengthOutOfRange(length);

            if (!_writable)
                StreamErrors.WriteNotSupported();

            if (length > _length)
                EnsureCapacity(length);

            _length = length;
            if (_position > length)
                _position = length;
        }

        public override void Seek(int position)
        {
            EnsureNotClosed();

            if (!CanSeek)
                StreamErrors.SeekNotSupported();

            _position = position;
        }

        public override void Write(byte[] buffer)
        {
            Write(buffer, 0, buffer.Length);
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            EnsureNotClosed();

            if (!CanWrite)
                StreamErrors.WriteNotSupported();

            if (offset < 0)
                throw new Exception("Offset out of range");
            if (count < 0)
                throw new Exception("Count out of range");

            int newPosition = _position + count;
            if (newPosition > _capacity)
                EnsureCapacity(newPosition);

            Array.Copy(buffer, offset, _data, _position, count);
            _position = newPosition;
            _length = Math.Max(_length, newPosition);
        }

        public override void WriteByte(byte value)
        {
            EnsureNotClosed();

            if (!CanWrite)
                StreamErrors.WriteNotSupported();

            int newPosition = _position + 1;
            if (newPosition > _capacity)
                EnsureCapacity(newPosition);

            _data[_position] = value;
            _position = newPosition;
            _length = Math.Max(_length, newPosition);
        }

        public override int Read(byte[] buffer, int offset, int count)
        {
            EnsureNotClosed();

            if (offset < 0)
                throw new Exception("Offset out of range");
            if (count < 0)
                throw new Exception("Count out of range");
            if (offset + count > buffer.Length)
                throw new Exception("Buffer destination is out of range");

            int startIndex = _position;
            int bytesToRead = Math.Min(count, _length - _position);
            if (bytesToRead <= 0)
                return 0; // nothing to read

            Array.Copy(_data, startIndex, buffer, offset, bytesToRead);
            _position = _position + bytesToRead;
            return bytesToRead;
        }

        public override int ReadByte()
        {
            EnsureNotClosed();
            if (_position > _length)
                return -1;

            int position = _position;
            _position++;
            return _data[position];
        }

        public byte[] ToArray()
        {
            EnsureNotClosed();

            byte[] bytes = new byte[_length];
            Array.Copy(_data, 0, bytes, 0, _length);
            return bytes;
        }
    }
}
