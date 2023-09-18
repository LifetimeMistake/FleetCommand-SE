using IngameScript.IO;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IngameScript.Network
{
    public struct NetMessageHandler
    {
        public delegate void NetMessageReceivedDelegate(NetInvocationContext context, BinaryReader reader);

        public NetMessageHandlerOptions Options;
        public NetMessageReceivedDelegate Callback;

        public NetMessageHandler(NetMessageHandlerOptions options, NetMessageReceivedDelegate callback)
        {
            Options = options;
            Callback = callback;
        }
    }
}
