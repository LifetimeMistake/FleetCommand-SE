using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public enum SystemNetMessage : ushort
    {
        AnnounceVessel = 0,
        AnnounceNetwork = 1,
    }
}
