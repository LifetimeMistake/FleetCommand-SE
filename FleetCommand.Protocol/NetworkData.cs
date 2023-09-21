using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class NetworkData
    {
        public long Id;
        public long? OwnerId;
        public List<long> Members;
        public DateTime LastSeen;
        public int MemberCount { get { return Members.Count; } }
        public NetworkData(long networkId, long? ownerId, DateTime lastSeen)
        {
            Id = networkId;
            OwnerId = ownerId;
            Members = new List<long>();
            LastSeen = lastSeen;
        }

        public NetworkData(long networkId, long? ownerId) : this(networkId, ownerId, DateTime.Now)
        { }
    }
}
