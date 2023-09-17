using System;

namespace IngameScript.IO
{
    public static class StreamErrors
    {
        public static void LengthOutOfRange(int length)
        {
            throw new Exception($"Stream length is out of range: {length}");
        }

        public static void StreamIsClosed()
        {
            throw new Exception("Cannot use closed stream.");
        }

        public static void SeekNotSupported()
        {
            throw new Exception("Stream does not support seeking.");
        }

        public static void ReadNotSupported()
        {
            throw new Exception("Stream does not support reading.");
        }

        public static void WriteNotSupported()
        {
            throw new Exception("Stream does not support writing.");
        }
    }
}
