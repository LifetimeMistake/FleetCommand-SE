using IngameScript.IO;
using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using VRage.Scripting;

namespace IngameScript.Network
{
    public class NetworkLink : IDisposable
    {
        public static Action<string> Log;
        private NetMessageHeader _header;
        private NetInvocationContext _context;
        private IMyIntergridCommunicationSystem _igc;
        private IMyBroadcastListener _broadcastListener;
        private IMyUnicastListener _unicastListener;

        private Stream _readBuffer;
        private Stream _writeBuffer;
        private BinaryReader _packetReader;
        private BinaryWriter _packetWriter;

        private string _channel;
        private Dictionary<ushort, NetMessageHandler> _listeners;
        private bool _disposed;

        public long OwnId { get; private set; }
        public long? OwnNetworkId { get; set; }

        public NetworkLink(Stream readBuffer, Stream writeBuffer, bool leaveOpen, string senderTag, IMyIntergridCommunicationSystem igc, long? networkId = null)
        {
            if (!readBuffer.CanRead && !readBuffer.CanWrite)
                StreamErrors.StreamIsClosed();

            if (!writeBuffer.CanRead && !writeBuffer.CanWrite)
                StreamErrors.StreamIsClosed();

            _header = new NetMessageHeader();
            _context = new NetInvocationContext(this, _header);
            _igc = igc;
            _broadcastListener = igc.RegisterBroadcastListener(senderTag);
            _unicastListener = igc.UnicastListener;

            _readBuffer = readBuffer;
            _writeBuffer = writeBuffer;
            _packetReader = new BinaryReader(_readBuffer, leaveOpen);
            _packetWriter = new BinaryWriter(_writeBuffer, leaveOpen);

            _channel = senderTag;
            _listeners = new Dictionary<ushort, NetMessageHandler>();

            OwnId = igc.Me;
            OwnNetworkId = networkId;
        }

        private ImmutableArray<byte> FillWriteBuffer(ISerializable data)
        {
            // Write to stream
            _writeBuffer.SetLength(0);
            _header.Serialize(_packetWriter);
            if (_header.HasData && data != null)
            {
                data.Serialize(_packetWriter);
            }

            _writeBuffer.Seek(0);

            // Convert to byte array
            int length = _writeBuffer.Length;
            byte[] buffer = new byte[length];
            _writeBuffer.Read(buffer, 0, length);
            return ImmutableArray.Create(buffer);
        }

        private void FillReadBuffer(ref ImmutableArray<byte> message)
        {
            byte[] data = new byte[message.Length];
            message.CopyTo(data);

            _readBuffer.Write(BitConverter.GetBytes((int)data.Length));
            _readBuffer.Write(data);
        }

        private void ReadPackets()
        {
            while (_unicastListener.HasPendingMessage)
            {
                MyIGCMessage packet = _unicastListener.AcceptMessage();
                if (!(packet.Data is ImmutableArray<byte>))
                    continue; // skip unknown packet

                ImmutableArray<byte> data = (ImmutableArray<byte>)packet.Data;
                FillReadBuffer(ref data);
            }

            while (_broadcastListener.HasPendingMessage)
            {
                MyIGCMessage packet = _broadcastListener.AcceptMessage();
                if (!(packet.Data is ImmutableArray<byte>))
                    continue; // skip unknown packet

                ImmutableArray<byte> data = (ImmutableArray<byte>)packet.Data;
                FillReadBuffer(ref data);
            }
        }

        private bool ProcessPacket()
        {
            try
            {
                NetMessageHeader header = NetMessageHeader.Deserialize(_packetReader);
                if (!_listeners.ContainsKey(header.Tag))
                {
                    Log($"Skipping unknown packet {header.Tag}");
                    return false; // Unknown packet
                }

                NetMessageHandler handler = _listeners[header.Tag];
                NetMessageHandlerOptions options = handler.Options;

                // Foreign broadcast: source network is not client network, dest network is client network, dest client is null
                if (
                    header.IsBroadcast &&
                    header.IsPrivate &&
                    header.SourceNetworkId != OwnNetworkId &&
                    header.DestinationNetworkId.HasValue &&
                    header.DestinationNetworkId == OwnNetworkId &&
                    !options.AcceptForeignBroadcasts)
                {
                    return false;
                }

                // Own broadcast: source network is client network, dest network is client network, dest client is null
                if (
                    header.IsBroadcast &&
                    header.IsPrivate &&
                    header.SourceNetworkId.HasValue &&
                    header.SourceNetworkId == OwnNetworkId &&
                    header.DestinationNetworkId.HasValue &&
                    header.DestinationNetworkId == OwnNetworkId &&
                    !options.AcceptOwnBroadcasts)
                {
                    return false;
                }

                // Public broadcast: dest network is null, dest client is null
                if (
                    header.IsBroadcast &&
                    header.IsPublic &&
                    !options.AcceptPublicBroadcasts)
                {
                    return false;
                }

                // Foreign unicast: source network is not client network, dest network is client network, dest client is local client
                if (
                    header.IsUnicast &&
                    header.IsPrivate &&
                    header.DestinationId.HasValue &&
                    header.DestinationId == OwnId &&
                    header.SourceNetworkId != OwnNetworkId &&
                    header.DestinationNetworkId.HasValue &&
                    header.DestinationNetworkId == OwnNetworkId &&
                    !options.AcceptForeignBroadcasts)
                {
                    return false;
                }

                // Own unicast: source network is client network, dest network is client network, dest client is local client
                if (
                    header.IsUnicast &&
                    header.IsPrivate &&
                    header.DestinationId.HasValue &&
                    header.DestinationId == OwnId &&
                    header.SourceNetworkId.HasValue &&
                    header.SourceNetworkId == OwnNetworkId &&
                    header.DestinationNetworkId.HasValue &&
                    header.DestinationNetworkId == OwnNetworkId &&
                    !options.AcceptOwnUnicasts)
                {
                    return false;
                }

                // Public unicast: dest network is null, dest client is local client
                if (
                    header.IsUnicast &&
                    header.IsPublic &&
                    header.DestinationId == OwnId &&
                    !options.AcceptPublicUnicasts)
                {
                    return false;
                }

                _context.Metadata = header;
                handler.Callback(_context, _packetReader);
                return true;
            }
            catch(Exception ex)
            {
                Log($"Aborting packet read: {ex}");
                return false;
            }
        }

        public void Dispose()
        {
            if (!_disposed)
            {
                _igc.DisableBroadcastListener(_broadcastListener);
                _igc.UnicastListener.DisableMessageCallback();
                _broadcastListener = null;
                _unicastListener = null;
                _igc = null;

                _packetReader.Dispose();
                _packetWriter.Dispose();
                _listeners.Clear();

                _readBuffer = null;
                _writeBuffer = null;
                _channel = null;
                _listeners = null;
                _disposed = true;
            }
        }

        public bool RegisterMessageHandler(ushort messageType, NetMessageHandler.NetMessageReceivedDelegate callback, NetMessageHandlerOptions options)
        {
            if (_listeners.ContainsKey(messageType))
                return false;

            NetMessageHandler handler = new NetMessageHandler(options, callback);
            _listeners[messageType] = handler;
            return true;
        }

        public bool RemoveMessageHandler(ushort messageType)
        {
            return _listeners.Remove(messageType);
        }

        public IEnumerable<NetMessageHandler> GetMessageHandlers()
        {
            return _listeners.Values;
        }

        /// <summary>
        /// Attempts to locate the specified peer
        /// </summary>
        /// <param name="peerId">The peer address to locate</param>
        /// <returns>The peer location relative to the current vessel. Returns <see cref="NetPeerLocation.Unknown"/> on failure.</returns>
        public NetPeerLocation GetPeerLocation(long peerId)
        {
            if (_igc.IsEndpointReachable(peerId, TransmissionDistance.CurrentConstruct))
            {
                return NetPeerLocation.Local;
            }

            if (_igc.IsEndpointReachable(peerId, TransmissionDistance.ConnectedConstructs) ||
                _igc.IsEndpointReachable(peerId, TransmissionDistance.TransmissionDistanceMax))
            {
                return NetPeerLocation.External;
            }

            return NetPeerLocation.Unknown;
        }

        /// <summary>
        /// Reads all incoming packets and routes them to registered message handlers
        /// Call this method frequency to make sure packet queues don't overflow
        /// <returns>The number of packets read this cycle</returns>
        /// </summary>
        public int ProcessPackets()
        {
            int count = 0;
            // Append packets to stream
            int readReturnPos = _readBuffer.Position;
            ReadPackets();
            _readBuffer.Seek(readReturnPos);

            while (_readBuffer.Position < _readBuffer.Length)
            {
                int length = _packetReader.ReadInt32();
                int skipLoc = _readBuffer.Position + length;

                if (!ProcessPacket())
                {
                    // Skip packet
                    _readBuffer.Seek(skipLoc);
                }

                count++;
            }

            // Reset buffer
            _readBuffer.SetLength(0);
            return count;
        }

        /// <summary>
        /// Sends a broadcast to all vessels in range regardless of network membership
        /// </summary>
        /// <param name="messageType">The message type</param>
        /// <param name="data">Additional data for message</param>
        public void SendPublicBroadcast(ushort messageType, ISerializable data)
        {
            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = null;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = null;
            _header.HasData = (data != null);
            _igc.SendBroadcastMessage(_channel, FillWriteBuffer(data));
        }

        /// <summary>
        /// Sends a broadcast to all vessels in the current network
        /// Fails if the current network link instance is not connected to any network
        /// </summary>
        /// <param name="messageType">The message type</param>
        /// <param name="data">Additional data for message</param>
        public bool SendNetworkBroadcast(ushort messageType, ISerializable data)
        {
            if (!OwnNetworkId.HasValue)
                return false;

            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = null;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = OwnNetworkId;
            _header.HasData = (data != null);
            _igc.SendBroadcastMessage(_channel, FillWriteBuffer(data));
            return true;
        }

        /// <summary>
        /// Sends a broadcast to all vessels in a specified target network
        /// </summary>
        /// <param name="networkId">Foreign network address</param>
        /// <param name="messageType">The message type</param>
        /// <param name="data">Additional data for message</param>
        public void SendForeignBroadcast(long networkId, ushort messageType, ISerializable data)
        {
            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = null;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = networkId;
            _header.HasData = (data != null);
            _igc.SendBroadcastMessage(_channel, FillWriteBuffer(data));
        }

        /// <summary>
        /// Sends a public message only to the specified vessel
        /// </summary>
        /// <param name="peerId">Recipient vessel address</param>
        /// <param name="messageType">The message type/param>
        /// <param name="data">Additional data for message</param>
        public void SendPublicUnicast(long peerId, ushort messageType, ISerializable data)
        {
            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = peerId;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = null;
            _header.HasData = (data != null);
            _igc.SendUnicastMessage(peerId, _channel, FillWriteBuffer(data));
        }

        /// <summary>
        /// Sends a message only to the specified vessel routing it through the current network
        /// </summary>
        /// <param name="peerId">Recipient vessel address</param>
        /// <param name="messageType">The message type/param>
        /// <param name="data">Additional data for message</param>
        public bool SendNetworkUnicast(long peerId, ushort messageType, ISerializable data)
        {
            if (!OwnNetworkId.HasValue)
                return false;

            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = peerId;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = OwnNetworkId;
            _header.HasData = (data != null);
            _igc.SendUnicastMessage(peerId, _channel, FillWriteBuffer(data));
            return true;
        }

        /// <summary>
        /// Sends a message only to the specified vessel routing it through the specified network
        /// </summary>
        /// <param name="peerId">Recipient vessel address</param>
        /// <param name="networkId">Recipient vessel network</param>
        /// <param name="messageType">The message type/param>
        /// <param name="data">Additional data for message</param>
        public void SendForeignUnicast(long peerId, long networkId, ushort messageType, ISerializable data)
        {
            _header.Tag = messageType;
            _header.SourceId = OwnId;
            _header.DestinationId = peerId;
            _header.SourceNetworkId = OwnNetworkId;
            _header.DestinationNetworkId = networkId;
            _header.HasData = (data != null);
            _igc.SendUnicastMessage(peerId, _channel, FillWriteBuffer(data));
        }
    }
}
