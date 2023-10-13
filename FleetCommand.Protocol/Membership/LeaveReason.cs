using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol.Membership
{
    public enum LeaveReason
    {
        Normal = 0,
        Kicked = 1,
        Error = 2
    }
}
