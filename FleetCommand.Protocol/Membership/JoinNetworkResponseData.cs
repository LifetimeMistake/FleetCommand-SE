using FleetCommand.Networking;
using FleetCommand.IO;
using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol.Membership
{
    public struct JoinNetworkResponseData : ISerializable
    {
        public long NetworkId;
        public JoinResult Result;

        public JoinNetworkResponseData(long networkId, JoinResult result)
        {
            NetworkId = networkId;
            Result = result;
        }

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(NetworkId);
            writer.Write((byte)Result);
        }

        public static JoinNetworkResponseData Deserialize(BinaryReader reader)
        {
            long networkId = reader.ReadInt64();
            JoinResult result = (JoinResult)reader.ReadByte();
            return new JoinNetworkResponseData(networkId, result);
        }
    }
}
