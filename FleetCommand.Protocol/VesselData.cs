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
        public DateTime LastSeen;

        public VesselData(long vesselId, long? networkId, DateTime lastSeen)
        {
            Id = vesselId;
            NetworkId = networkId;
            LastSeen = lastSeen;
        }

        public VesselData(long vesselId, long? networkId) : (vesselId, networkId, DateTime.Now)
        { }
    }
}
