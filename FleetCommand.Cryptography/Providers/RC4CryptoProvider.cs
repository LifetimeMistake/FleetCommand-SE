using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Cryptography.Providers
{
    public class RC4CryptoProvider : ICryptoProvider
    {
        private byte[] _key;
        private byte[] _state;
        private int _i;
        private int _j;

        public RC4CryptoProvider(byte[] key)
        {
            _key = key;
            InitializeState();
        }

        private void InitializeState()
        {
            _state = new byte[256];
            for (int i = 0; i < 256; i++)
            {
                _state[i] = (byte)i;
            }

            int j = 0;
            for (int i = 0; i < 256; i++)
            {
                j = (j + _state[i] + _key[i % _key.Length]) % 256;
                Swap(_state, i, j);
            }

            _i = 0;
            _j = 0;
        }

        private void Swap(byte[] array, int i, int j)
        {
            byte temp = array[i];
            array[i] = array[j];
            array[j] = temp;
        }

        public void Encrypt(byte[] data, int offset, int count)
        {
            for (int k = offset; k < offset + count; k++)
            {
                _i = (_i + 1) % 256;
                _j = (_j + _state[_i]) % 256;
                Swap(_state, _i, _j);
                byte keyByte = _state[(_state[_i] + _state[_j]) % 256];
                data[k] ^= keyByte;
            }
        }

        public void Decrypt(byte[] data, int offset, int count)
        {
            Encrypt(data, offset, count);
        }
    }

}
