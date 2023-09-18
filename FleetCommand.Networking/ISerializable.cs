using FleetCommand.IO;

namespace FleetCommand.Networking
{
    public interface ISerializable
    {
        void Serialize(BinaryWriter writer);
    }
}
