using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public enum SystemNetMessage : ushort
    {
        AnnounceVessel = 0,
        AnnounceNetwork = 1,
        ElevateOwner = 2,
        JoinNetwork = 3,
        NetworkJoinResponse = 4,
        LeaveNetwork = 5,
        MemberJoined = 6,
        MemberKicked = 7
    }
}
