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
        public Network NetworkData;

        public bool HasNetwork { get { return NetworkId.HasValue; } }
        public bool NetworkDataAvailable { get { return NetworkId.HasValue && NetworkData != null; } }
        public bool OwnsNetwork { get { return NetworkId.HasValue && NetworkData.OwnerId == Id; } }
        public bool IsTrusted { get { return !NetworkId.HasValue || (NetworkData != null && NetworkData.Id == NetworkId);  } }

        public Vessel(long vesselId, int lastSeen)
        {
            Id = vesselId;
            NetworkId = null;
            LastSeen = lastSeen;
            NetworkData = null;
        }

        public Vessel(long vesselId, int lastSeen, long networkId)
        {
            Id = vesselId;
            NetworkId = networkId;
            LastSeen = lastSeen;
            NetworkData = null;
        }

        public Vessel(long vesselId, int lastSeen, Network network)
        {
            if (network == null)
                throw new ArgumentException("Vessel is part of a network but no network data has been supplied.");

            Id = vesselId;
            NetworkId = network.Id;
            LastSeen = lastSeen;
            NetworkData = network;
        }
    }
}
