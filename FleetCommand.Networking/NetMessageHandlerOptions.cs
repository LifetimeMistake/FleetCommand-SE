namespace FleetCommand.Networking
{
    public struct NetMessageHandlerOptions
    {
        public bool AcceptPublicBroadcasts;
        public bool AcceptPublicUnicasts;

        public bool AcceptOwnBroadcasts;
        public bool AcceptOwnUnicasts;

        public bool AcceptForeignBroadcasts;
        public bool AcceptForeignUnicasts;

        public static NetMessageHandlerOptions CreatePublicHandler(bool acceptBroadcasts, bool acceptUnicasts)
        {
            NetMessageHandlerOptions options = new NetMessageHandlerOptions();
            options.AcceptPublicBroadcasts = acceptBroadcasts;
            options.AcceptPublicUnicasts = acceptUnicasts;
            return options;
        }

        public static NetMessageHandlerOptions CreateOwnHandler(bool acceptBroadcasts, bool acceptUnicasts)
        {
            NetMessageHandlerOptions options = new NetMessageHandlerOptions();
            options.AcceptOwnBroadcasts = acceptBroadcasts;
            options.AcceptOwnUnicasts = acceptUnicasts;
            return options;
        }

        public static NetMessageHandlerOptions CreateForeignHandler(bool acceptBroadcasts, bool acceptUnicasts)
        {
            NetMessageHandlerOptions options = new NetMessageHandlerOptions();
            options.AcceptForeignBroadcasts = acceptBroadcasts;
            options.AcceptForeignUnicasts = acceptUnicasts;
            return options;
        }
    }
}
