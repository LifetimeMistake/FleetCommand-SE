using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public struct VesselBeacon : ISerializable
    {
        public long VesselId;
        public long? NetworkId;

        public VesselBeacon(long vesselId, long? networkId)
        {
            VesselId = vesselId;
            NetworkId = networkId;
        }

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(VesselId);
            writer.Write(NetworkId.HasValue);
            if (NetworkId.HasValue) 
                writer.Write(NetworkId.Value);
        }

        public static VesselBeacon Deserialize(BinaryReader reader)
        {
            long vesselId = reader.ReadInt64();
            bool hasNetwork = reader.ReadBoolean();
            long? networkId = (hasNetwork ? (long?)reader.ReadInt64() : null);
            return new VesselBeacon(vesselId, networkId);
        }
    }
}
