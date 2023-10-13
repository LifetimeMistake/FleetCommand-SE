using System;

namespace FleetCommand.Cryptography.Providers
{
    public class XORCryptoProvider : ICryptoProvider
    {
        private byte[] _key;

        public XORCryptoProvider(byte[] key)
        {
            _key = key;
        }

        public void Encrypt(byte[] data, int offset, int count)
        {
            int end = Math.Min(data.Length, offset + count);
            for (int i = offset; i < end; i++)
                data[i] = (byte)(data[i] ^ _key[i % _key.Length]);
        }

        public void Decrypt(byte[] data, int offset, int count)
        {
            int end = Math.Min(data.Length, offset + count);
            for (int i = offset; i < end; i++)
                data[i] = (byte)(_key[i % _key.Length] ^ data[i]);
        }
    }
}
