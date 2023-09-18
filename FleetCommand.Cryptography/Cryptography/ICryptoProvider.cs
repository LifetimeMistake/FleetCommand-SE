namespace FleetCommand.Cryptography
{
    public interface ICryptoProvider
    {
        void Encrypt(byte[] buffer, int offset, int count);
        void Decrypt(byte[] buffer, int offset, int count);
    }
}
