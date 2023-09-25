using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

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
                Network network = new Network(beacon.NetworkId, beacon.OwnerId, _timekeeper.Now);
                UpdateMembership(network, beacon.Members);
                network.Members = beacon.Members;

                _networks.Add(network);
                OnNetworkDiscovered?.Invoke(network);
            }
            else
            {
                // Network is known already
                Network network = _networks.Get(beacon.NetworkId);
                network.OwnerId = beacon.OwnerId;

                if (!network.Members.SequenceEqual(beacon.Members))
                {
                    UpdateMembership(network, beacon.Members);
                    network.Members = beacon.Members;
                }

                network.LastSeen = _timekeeper.Now;
            }
        }

        public void Update()
        {
            int now = _timekeeper.Now;

            foreach (Network network in _networks)
            {
                if ((now - network.LastSeen) >= _networkTimeout)
                {
                    _dropped.Add(network);
                }
            }

            if (_dropped.Count > 0)
            {
                foreach(Network network in _dropped)
                {
                    UpdateMembership(network, _nullMembers);
                    _networks.Remove(network.Id);
                    OnNetworkLost?.Invoke(network);
                }

                _dropped.Clear();
            }

            if ((now - _lastAnnounce) >= _announceInterval && _localVessel.OwnsNetwork && _localVessel.NetworkDataAvailable)
            {
                Network network = _localVessel.NetworkData;
                NetworkBeacon beacon = new NetworkBeacon(network.Id, network.OwnerId, network.Members);
                _link.SendPublicBroadcast((ushort)SystemNetMessage.AnnounceNetwork, beacon);
                _lastAnnounce = now;
            }
        }

        /// <summary>
        /// Updates network data in member vessels
        /// Does not update vessels that do not declare themselves as part of the network
        /// even if the <paramref name="members"/> list contains the vessel.
        /// This satisfies the mutual distrust of vessels and networks
        /// </summary>
        /// <param name="network">Target network</param>
        /// <param name="members">Updated list of members</param>
        private void UpdateMembership(Network network, List<long> members)
        {
            foreach(Vessel vessel in _vessels)
            {
                if (vessel.NetworkId != network.Id)
                    continue;

                vessel.NetworkData = members.Contains(vessel.Id) ? network : null;
            }
        }
    }
}
