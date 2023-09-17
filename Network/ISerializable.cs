using IngameScript.IO;

namespace IngameScript.Network
{
    public interface ISerializable
    {
        void Serialize(BinaryWriter writer);
    }
}
