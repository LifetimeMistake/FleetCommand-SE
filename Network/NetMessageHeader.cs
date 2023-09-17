using IngameScript.IO;
using System.Text;

namespace IngameScript.Network
{
    public struct NetMessageHeader : ISerializable
    {
        public static byte[] HEADER_ID = Encoding.ASCII.GetBytes("FCNET");
        public ushort Tag;
        public long SourceId;
        public long? DestinationId;
        public long? SourceNetworkId;
        public long? DestinationNetworkId;
        public bool HasData;

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(HEADER_ID);
            writer.Write(SourceId);
            writer.Write(Tag);

            writer.Write(DestinationId.HasValue);
            if (DestinationId.HasValue)
                writer.Write(DestinationId.Value);

            writer.Write(SourceNetworkId.HasValue);
            if (SourceNetworkId.HasValue)
                writer.Write(SourceNetworkId.Value);

            writer.Write(DestinationNetworkId.HasValue);
            if (DestinationNetworkId.HasValue)
                writer.Write(DestinationNetworkId.Value);

            writer.Write(HasData);
        }
    }
}
