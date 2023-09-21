using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class VesselDiscoveryData
    {
        public long VesselId;
        public DateTime LastSeen;

        public VesselDiscoveryData(long vesselId, DateTime lastSeen)
        {
            VesselId = vesselId;
            LastSeen = lastSeen;
        }
    }
}
