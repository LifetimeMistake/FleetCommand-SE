﻿using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol.Membership
{
    public enum JoinResult : byte
    {
        OK = 0,
        AlreadyAuthenticated = 1,
        Timeout = 2,
        UnknownReject = 3
    }
}
