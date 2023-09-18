namespace IngameScript.Network
{
    public class NetInvocationContext
    {
        public NetworkLink Network;
        public NetMessageHeader Metadata;
        public NetPeerLocation SourceLocation
        {
            get
            {
                return Network.GetPeerLocation(Metadata.SourceId);
            }
        }

        public NetInvocationContext(NetworkLink network, NetMessageHeader header)
        {
            Network = network;
            Metadata = header;
        }
    }
}
