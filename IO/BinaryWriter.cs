using System;
using System.Text;
using VRageMath;

namespace IngameScript.IO
{
    public class BinaryWriter : IDisposable
    {
        private const int BUFFER_SIZE = 16;
        private Stream _stream;
        private byte[] _buffer;
        private bool _leaveOpen;
        private Encoding _encoding;

        public BinaryWriter(Stream stream, bool leaveOpen) : this(stream, new UTF8Encoding(false, true), leaveOpen)
        { }

        public BinaryWriter(Stream stream, Encoding encoding, bool leaveOpen)
        {
            _stream = stream;
            _buffer = new byte[BUFFER_SIZE];
            _leaveOpen = leaveOpen;
            _encoding = encoding;
        }

        public virtual void Close()
        {
            Dispose(true);
        }

        public void Dispose()
        {
            Dispose(true);
        }

        public virtual void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (_stream != null)
                {
                    _stream.Flush();

                    if (!_leaveOpen)
                    {
                        _stream.Close();
                    }
                }

                _stream = null;
                _buffer = null;
                _encoding = null;
            }
        }

        public virtual void Flush()
        {
            _stream.Flush();
        }

        public virtual void Seek(int position)
        {
            _stream.Seek(position);
        }

        public virtual void Write(bool value)
        {
            Write((byte)(value ? 1 : 0));
        }

        public virtual void Write(byte value)
        {
            _stream.WriteByte(value);
        }

        public virtual void Write(sbyte value)
        {
            Write((byte)value);
        }

        public virtual void Write(byte[] buffer)
        {
            _stream.Write(buffer);
        }

        public virtual void Write(byte[] buffer, int offset, int count)
        {
            _stream.Write(buffer, offset, count);
        }

        public virtual void Write(float value)
        {
            _stream.Write(BitConverter.GetBytes(value));
        }

        public virtual void Write(double value)
        {
            _stream.Write(BitConverter.GetBytes(value));
        }

        public virtual void Write(short value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _stream.Write(_buffer, 0, 2);
        }

        public virtual void Write(ushort value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _stream.Write(_buffer, 0, 2);
        }

        public virtual void Write(int value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _buffer[2] = (byte)(value >> 16);
            _buffer[3] = (byte)(value >> 24);
            _stream.Write(_buffer, 0, 4);
        }

        public virtual void Write(uint value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _buffer[2] = (byte)(value >> 16);
            _buffer[3] = (byte)(value >> 24);
            _stream.Write(_buffer, 0, 4);
        }

        public virtual void Write(long value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _buffer[2] = (byte)(value >> 16);
            _buffer[3] = (byte)(value >> 24);
            _buffer[4] = (byte)(value >> 32);
            _buffer[5] = (byte)(value >> 40);
            _buffer[6] = (byte)(value >> 48);
            _buffer[7] = (byte)(value >> 56);
            _stream.Write(_buffer, 0, 8);
        }

        public virtual void Write(ulong value)
        {
            _buffer[0] = (byte)value;
            _buffer[1] = (byte)(value >> 8);
            _buffer[2] = (byte)(value >> 16);
            _buffer[3] = (byte)(value >> 24);
            _buffer[4] = (byte)(value >> 32);
            _buffer[5] = (byte)(value >> 40);
            _buffer[6] = (byte)(value >> 48);
            _buffer[7] = (byte)(value >> 56);
            _stream.Write(_buffer, 0, 8);
        }

        public virtual void Write(string value)
        {
            if (value == null)
            {
                Write(false); // has no content
            }
            else
            {
                Write(true);
                byte[] bytes = _encoding.GetBytes(value);
                Write((uint)bytes.Length);
                Write(bytes);
            }
        }

        public virtual void Write(Vector3 vector)
        {
            Write(vector.X);
            Write(vector.Y);
            Write(vector.Z);
        }

        public virtual void WriteQuaternion(Quaternion quaternion)
        {
            Write(quaternion.X);
            Write(quaternion.Y);
            Write(quaternion.Z);
            Write(quaternion.W);
        }
    }
}
