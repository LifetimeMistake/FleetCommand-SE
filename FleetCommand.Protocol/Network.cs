using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class Network
    {
        public long Id;
        public long? OwnerId;
        public HashSet<long> Members;
        public int LastSeen;

        public int MemberCount => Members.Count;
        public bool HasOwner => OwnerId != null;

        public Network(long networkId, long? ownerId, int lastSeen)
        {
            Id = networkId;
            OwnerId = ownerId;
            Members = new HashSet<long>();
            LastSeen = lastSeen;

            if (ownerId.HasValue)
                Members.Add(ownerId.Value);
        }

        public Network(long networkId, long? ownerId, int lastSeen, HashSet<long> members)
        {
            Id = networkId;
            OwnerId = ownerId;
            Members = members;
            LastSeen = lastSeen;

            if (ownerId.HasValue)
                Members.Add(ownerId.Value); // just to make sure
        }

        public NetworkRelationship GetRelationship(Vessel vessel)
        {
            if (Members.Contains(vessel.Id))
            {
                if (vessel.NetworkId == Id)
                {
                    return NetworkRelationship.Member;
                }
                else
                {
                    return NetworkRelationship.Authenticated;
                }
            }
            else
            {
                return NetworkRelationship.Unauthenticated;
            }
        }
    }
}
