using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Library.Collections;

namespace IngameScript.IO
{
    public class BinaryReader : IDisposable
    {
        private const int BUFFER_SIZE = 16;
        private Stream _stream;
        private byte[] _buffer;
        private bool _leaveOpen;
        private Encoding _encoding;

        public BinaryReader(Stream stream, bool leaveOpen) : this(stream, new UTF8Encoding(false, true), leaveOpen)
        { }

        public BinaryReader(Stream stream, Encoding encoding, bool leaveOpen)
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
                if (_stream != null && !_leaveOpen)
                {
                    _stream.Close();
                }

                _stream = null;
                _buffer = null;
                _encoding = null;
            }
        }

        protected virtual void FillBuffer(int numBytes)
        {
            if (_buffer != null && (numBytes < 0 || numBytes > _buffer.Length))
                throw new ArgumentException(nameof(numBytes), "Requested number of bytes is out of range");

            if (_stream == null)
                StreamErrors.StreamIsClosed();

            if (numBytes == 1)
            {
                int value = _stream.ReadByte();
                if (value == -1)
                    StreamErrors.EndOfStream();

                _buffer[0] = (byte)value;
                return;
            }

            int totalRead = 0;
            do
            {
                int bytesRead = _stream.Read(_buffer, totalRead, numBytes - totalRead);
                if (bytesRead == 0)
                    StreamErrors.EndOfStream();

                totalRead += bytesRead;
            }
            while (totalRead < numBytes);
        }

        public virtual bool ReadBoolean()
        {
            FillBuffer(1);
            return _buffer[0] > 0;
        }

        public virtual byte ReadByte()
        {
            FillBuffer(1);
            return _buffer[0];
        }

        public virtual sbyte ReadSByte()
        {
            FillBuffer(1);
            return (sbyte)_buffer[0];
        }

        public virtual byte[] ReadBytes(int numBytes)
        {
            if (numBytes < 0)
                throw new ArgumentException(nameof(numBytes), "Number of bytes must be positive");

            if (_stream == null)
                StreamErrors.StreamIsClosed();

            if (numBytes == 0)
                return new byte[] {};

            byte[] buffer = new byte[numBytes];
            int totalRead = 0;
            do
            {
                int bytesRead = _stream.Read(buffer, totalRead, numBytes - totalRead);
                if (bytesRead == 0)
                    StreamErrors.EndOfStream();

                totalRead += bytesRead;
            }
            while (totalRead < numBytes);

            return buffer;
        }

        public virtual float ReadSingle()
        {
            FillBuffer(4);
            return BitConverter.ToSingle(_buffer, 0);
        }

        public virtual double ReadDouble()
        {
            FillBuffer(8);
            return BitConverter.ToDouble(_buffer, 0);
        }

        public virtual short ReadInt16()
        {
            FillBuffer(2);
            return (short)((int)_buffer[0] | ((int)_buffer[1] << 8));
        }

        public virtual ushort ReadUInt16()
        {
            FillBuffer(2);
            return (ushort)((int)_buffer[0] | ((int)_buffer[1] << 8));
        }

        public virtual int ReadInt32()
        {
            FillBuffer(4);
            return (int)((int)_buffer[0] | ((int)_buffer[1] << 8) | ((int)_buffer[2] << 16) | ((int)_buffer[3] << 24));
        }

        public virtual uint ReadUInt32()
        {
            FillBuffer(4);
            return (uint)((int)_buffer[0] | ((int)_buffer[1] << 8) | ((int)_buffer[2] << 16) | ((int)_buffer[3] << 24));
        }

        public virtual long ReadInt64()
        {
            FillBuffer(8);
            uint high = (uint)((int)_buffer[0] | ((int)_buffer[1] << 8) | ((int)_buffer[2] << 16) | ((int)_buffer[3] << 24));
            uint low = (uint)((int)_buffer[4] | ((int)_buffer[5] << 8) | ((int)_buffer[6] << 16) | ((int)_buffer[7] << 24));
            return (long)(((ulong)low << 32) | (ulong)high);
        }

        public virtual ulong ReadUInt64()
        {
            FillBuffer(8);
            uint high = (uint)((int)_buffer[0] | ((int)_buffer[1] << 8) | ((int)_buffer[2] << 16) | ((int)_buffer[3] << 24));
            uint low = (uint)((int)_buffer[4] | ((int)_buffer[5] << 8) | ((int)_buffer[6] << 16) | ((int)_buffer[7] << 24));
            return (((ulong)low << 32) | (ulong)high);
        }

        public virtual string ReadString()
        {
            bool hasContent = ReadBoolean();
            if (!hasContent)
                return null;

            uint length = ReadUInt32();
            byte[] bytes = ReadBytes((int)length);
            return _encoding.GetString(bytes);
        }
    }
}
