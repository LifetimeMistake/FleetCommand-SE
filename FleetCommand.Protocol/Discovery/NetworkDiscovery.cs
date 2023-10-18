using EmptyKeys.UserInterface.Generated.DataTemplatesContracts_Bindings;
using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Net;

namespace FleetCommand.Protocol.Discovery
{
    public class NetworkDiscovery
    {
        // optimizations to reduce heap allocations during ticks
        private List<long> _nullMembers = new List<long>();
        private List<Network> _dropped = new List<Network>();
        private NetworkLink _link;
        private Vessel _localVessel;
        private Vessels _vessels;
        private Networks _networks;
        private Timekeeper _timekeeper;

        private int _networkTimeout;
        private int _announceInterval;
        private int _lastAnnounce;

        public delegate void NetworkDiscoveryDelegate(Network network);
        public event NetworkDiscoveryDelegate OnNetworkDiscovered;
        public event NetworkDiscoveryDelegate OnNetworkLost;

        public NetworkDiscovery(NetworkLink link, Vessels vessels, Networks networks, Timekeeper timekeeper, float networkTimeout, float announceInterval)
        {
            _link = link;
            _vessels = vessels;
            _networks = networks;
            _localVessel = vessels.GetLocalVessel();
            _timekeeper = timekeeper;
            _networkTimeout = timekeeper.SecondsToTicks(networkTimeout);
            _announceInterval = timekeeper.SecondsToTicks(announceInterval);
            _lastAnnounce = 0;

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.AnnounceNetwork, ReceiveAnnounceNetwork, NetMessageHandlerOptions.CreatePublicHandler(true, false)))
                throw new Exception("Could not register a message handler for AnnounceNetwork");
        }

        private void ReceiveAnnounceNetwork(NetInvocationContext context, BinaryReader reader)
        {
            NetMessageHeader header = context.Metadata;
            if (!header.HasData)
                return; // TODO: log error

            NetworkBeacon beacon = NetworkBeacon.Deserialize(reader);

            if (beacon.NetworkId == _localVessel.NetworkId)
                return; // do not update local network here

            if (!_networks.Contains(beacon.NetworkId))
            {
                // Network just got discovered
                Network network = new Network(beacon.NetworkId, beacon.OwnerId, _timekeeper.Now, beacon.Members);

                _networks.Add(network);
                OnNetworkDiscovered?.Invoke(network);
            }
            else
            {
                // Network is known already
                Network network = _networks.Get(beacon.NetworkId);
                network.OwnerId = beacon.OwnerId;
                network.Members = beacon.Members;
                network.LastSeen = _timekeeper.Now;
            }
        }

        public void Update()
        {
            int now = _timekeeper.Now;

            foreach (Network network in _networks)
            {
                // Do not drop local network
                if (network.Id == _localVessel.NetworkId)
                {
                    network.LastSeen = now;
                    continue;
                }

                if ((now - network.LastSeen) >= _networkTimeout)
                {
                    _dropped.Add(network);
                }
            }

            if (_dropped.Count > 0)
            {
                foreach(Network network in _dropped)
                {
                    _networks.Remove(network.Id);
                    OnNetworkLost?.Invoke(network);
                }

                _dropped.Clear();
            }

            if ((now - _lastAnnounce) >= _announceInterval)
            {
                Network network = _networks.GetLocalNetwork();
                if (network == null || _localVessel.Id != network.OwnerId)
                    return;

                NetworkBeacon beacon = new NetworkBeacon(network.Id, network.OwnerId, network.Members);
                _link.SendPublicBroadcast((ushort)SystemNetMessage.AnnounceNetwork, beacon);
                _lastAnnounce = now;
            }
        }
    }
}
