using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Security.Cryptography.X509Certificates;
using System.Text;

namespace FleetCommand.Protocol
{
    public struct NetworkBeacon : ISerializable
    {
        public long NetworkId;
        public long? OwnerId;
        public HashSet<long> Members;
        public int MemberCount { get {  return Members.Count; } }

        public NetworkBeacon(long networkId, long? ownerId, HashSet<long> members)
        {
            NetworkId = networkId;
            OwnerId = ownerId;
            Members = members;
        }

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(NetworkId);
            writer.Write(MemberCount);
            writer.Write(OwnerId.HasValue);

            if (OwnerId.HasValue)
                writer.Write(OwnerId.Value);

            foreach (var member in Members)
                writer.Write(member);

        }

        public static NetworkBeacon Deserialize(BinaryReader reader)
        {
            long networkId = reader.ReadInt64();
            int memberCount = reader.ReadInt32();
            bool hasOwner = reader.ReadBoolean();

            long? ownerId = (hasOwner ? (long?)reader.ReadInt64() : null);

            HashSet<long> members = new HashSet<long>();
            for (int i = 0; i < memberCount; i++)
                members.Add(reader.ReadInt64());

            return new NetworkBeacon(networkId, ownerId, members);
        }
    }
}
