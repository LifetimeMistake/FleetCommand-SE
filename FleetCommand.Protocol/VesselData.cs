using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class VesselData
    {
        public long Id;
        public long? NetworkId;
        public bool Verified;
        public int LastSeen;

        public VesselData(long vesselId, long? networkId, int lastSeen)
        {
            Id = vesselId;
            NetworkId = networkId;
            LastSeen = lastSeen;
        }
    }
}
