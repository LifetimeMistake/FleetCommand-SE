using FleetCommand.IO;

namespace FleetCommand.Networking
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
