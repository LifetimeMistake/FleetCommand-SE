using FleetCommand.Cryptography;
using FleetCommand.Cryptography.Providers;
using System;

namespace FleetCommand.IO
{
    public class EncryptedStream : Stream
    {
        private ICryptoProvider _crypto;
        private Stream _stream;

        public override int Position
        {
            get
            {
                EnsureNotClosed();
                return _stream.Position;
            }

            set
            {
                EnsureNotClosed();
                _stream.Position = value;
            }
        }

        public override int Capacity
        {
            get
            {
                EnsureNotClosed();
                return _stream.Capacity;
            }

            set
            {
                EnsureNotClosed();
                _stream.Capacity = value;
            }
        }

        public override int Length
        {
            get
            {
                EnsureNotClosed();
                return _stream.Length;
            }
        }

        public override bool CanSeek
        {
            get
            {
                return _stream != null && _stream.CanSeek;
            }
        }

        public override bool CanRead
        {
            get
            {
                return _stream != null && _stream.CanRead;
            }
        }

        public override bool CanWrite
        {
            get
            {
                return _stream != null && _stream.CanWrite;
            }
        }

        public EncryptedStream(Stream innerStream, ICryptoProvider cryptoProvider)
        {
            if (innerStream == null)
                throw new ArgumentNullException(nameof(innerStream));

            _stream = innerStream;
            _crypto = cryptoProvider;

            if (!_stream.CanRead && !_stream.CanWrite)
                StreamErrors.StreamIsClosed();
        }

        private void EnsureNotClosed()
        {
            if (_stream == null)
                StreamErrors.StreamIsClosed();
        }

        public override void Dispose(bool disposing)
        {
            if (disposing && _stream != null)
            {
                try
                {
                    Flush();
                }
                finally
                {
                    _stream.Close();
                }
            }

            _stream = null;
            _crypto = null;
            base.Dispose(disposing);
        }

        public override void SetLength(int length)
        {
            _stream.SetLength(length);
        }

        public override void Seek(int position)
        {
            _stream.Seek(position);
        }

        public override void Flush()
        {
            _stream.Flush();
        }

        public override void Write(byte[] buffer)
        {
            Write(buffer, 0, buffer.Length);
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            if (_stream == null)
                StreamErrors.StreamIsClosed();

            if (!CanWrite)
                StreamErrors.WriteNotSupported();

            byte[] encrypted = new byte[count];
            Array.Copy(buffer, offset, encrypted, 0, count);
            _crypto.Encrypt(encrypted, 0, count);
            WriteEncrypted(encrypted, 0, count);
        }

        public void WriteEncrypted(byte[] encrypted, int offset, int count)
        {
            if (_stream == null)
                StreamErrors.StreamIsClosed();

            _stream.Write(encrypted, offset, count);
        }

        public override int Read(byte[] buffer, int offset, int count)
        {
            if (_stream == null)
                StreamErrors.StreamIsClosed();

            int bytesRead = _stream.Read(buffer, offset, count);
            _crypto.Decrypt(buffer, offset, bytesRead);
            return bytesRead;
        }

        public int ReadEncrypted(byte[] buffer, int offset, int count)
        {
            if (_stream == null)
                StreamErrors.StreamIsClosed();

            return _stream.Read(buffer, offset, count);
        }
    }
}
