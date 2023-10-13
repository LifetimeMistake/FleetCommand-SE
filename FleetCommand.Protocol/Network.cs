using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class Network
    {
        public long Id;
        public long? OwnerId;
        public List<long> Members;
        public int LastSeen;

        public int MemberCount { get { return Members.Count; } }
        public bool HasOwner { get { return OwnerId != null; } }

        public Network(long networkId, long? ownerId, int lastSeen)
        {
            Id = networkId;
            OwnerId = ownerId;
            Members = new List<long>();
            LastSeen = lastSeen;

            if (ownerId.HasValue)
                Members.Add(ownerId.Value);
        }
    }
}
