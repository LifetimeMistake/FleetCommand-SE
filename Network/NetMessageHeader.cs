using IngameScript.IO;
using System;
using System.Linq;
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

        public NetMessageHeader(ushort tag, long sourceId, long? destinationId, long? sourceNetworkId, long? destinationNetworkId, bool hasData)
        {
            Tag = tag;
            SourceId = sourceId;
            DestinationId = destinationId;
            SourceNetworkId = sourceNetworkId;
            DestinationNetworkId = destinationNetworkId;
            HasData = hasData;
        }

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(HEADER_ID);
            writer.Write(Tag);
            writer.Write(SourceId);

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

        public static NetMessageHeader Deserialize(BinaryReader reader)
        {
            byte[] headerID = reader.ReadBytes(HEADER_ID.Length);
            if (!headerID.SequenceEqual(HEADER_ID))
                throw new Exception("Data is not a valid network message header");

            ushort tag = reader.ReadUInt16();
            long sourceId = reader.ReadInt64();
            long? destinationId = (reader.ReadBoolean() ? (long?)reader.ReadInt64() : null);
            long? sourceNetworkId = (reader.ReadBoolean() ? (long?)reader.ReadInt64() : null);
            long? destinatioNnetworkId = (reader.ReadBoolean() ? (long?)reader.ReadInt64() : null);
            bool hasData = reader.ReadBoolean();

            return new NetMessageHeader(tag, sourceId, destinationId, sourceNetworkId, destinatioNnetworkId, hasData);
        }
    }
}
