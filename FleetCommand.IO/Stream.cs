using System;

namespace FleetCommand.IO
{
    public abstract class Stream : IDisposable
    {
        public abstract int Position { get; set; }
        public abstract int Length { get; }
        public abstract int Capacity { get; set; }
        public abstract bool CanRead { get; }
        public abstract bool CanSeek { get; }
        public abstract bool CanWrite { get; }

        public abstract void Seek(int position);
        public abstract void SetLength(int length);
        public virtual void WriteByte(byte value)
        {
            Write(new byte[] { value }, 0, 1);
        }
        public abstract void Write(byte[] buffer, int offset, int count);
        public abstract void Write(byte[] buffer);
        public virtual int ReadByte()
        {
            byte[] array = new byte[1];
            if (Read(array, 0, 1) == 0)
            {
                return -1;
            }
            return array[0];
        }
        public abstract int Read(byte[] buffer, int offset, int count);
        public abstract void Flush();
        public virtual void Close()
        {
            this.Dispose(true);
        }
        public void Dispose()
        {
            this.Close();
        }
        public virtual void Dispose(bool disposing) { }
    }
}
