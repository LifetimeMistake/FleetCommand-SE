using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class Vessel
    {
        public long Id;
        public long? NetworkId;
        public int LastSeen;

        public bool HasNetwork { get { return NetworkId.HasValue; } }

        public Vessel(long vesselId, long? networkId, int lastSeen)
        {
            Id = vesselId;
            NetworkId = networkId;
            LastSeen = lastSeen;
        }
    }
}
