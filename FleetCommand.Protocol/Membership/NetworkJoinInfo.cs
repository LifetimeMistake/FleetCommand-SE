using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol.Membership
{
    public struct NetworkJoinInfo
    {
        public long NetworkId;
        public int ExpirationTime;

        public NetworkJoinInfo(long networkId, int expirationTime)
        {
            NetworkId = networkId;
            ExpirationTime = expirationTime;
        }
    }
}
