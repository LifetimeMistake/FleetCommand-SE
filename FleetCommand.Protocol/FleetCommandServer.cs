using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace FleetCommand.Protocol
{
    public class FleetCommandServer
    {
        private NetworkLink _link;
        private ServerOptions _config;
        private VesselData _self;
        private int _lastVesselAnnounce;
        private NetworkData _selfNet;
        private int _lastNetworkAnnounce;
        private Dictionary<long, VesselData> _knownVessels;
        private Dictionary<long, NetworkData> _knownNetworks;
        private Timekeeper _timekeeper;

        private int _vesselTimeoutTicks;
        private int _networkTimeoutTicks;
        private int _announceIntervalTicks;

        public delegate void VesselDelegate(VesselData vessel);
        public delegate void NetworkDiscoveryDelegate(NetworkData net);

        public event VesselDelegate OnVesselDiscovered;
        public event VesselDelegate OnVesselLost;
        public event NetworkDiscoveryDelegate OnNetworkDiscovered;
        public event NetworkDiscoveryDelegate OnNetworkLost;

        public FleetCommandServer(NetworkLink link, Timekeeper timekeeper, ServerOptions options)
        {
            _link = link;
            _config = options;
            _knownVessels = new Dictionary<long, VesselData>();
            _knownNetworks = new Dictionary<long, NetworkData>();

            _self = new VesselData(link.OwnId, link.OwnNetworkId, timekeeper.Now);
            _self.Verified = true;
            _knownVessels.Add(link.OwnId, _self);

            if (link.OwnNetworkId.HasValue)
            {
                _selfNet = new NetworkData(link.OwnNetworkId.Value, null, timekeeper.Now);
                _knownNetworks.Add(_selfNet.Id, _selfNet);
            }

            _timekeeper = timekeeper;
            _vesselTimeoutTicks = timekeeper.SecondsToTicks(options.VesselTimeout);
            _networkTimeoutTicks = timekeeper.SecondsToTicks(options.NetworkTimeout);
            _announceIntervalTicks = timekeeper.SecondsToTicks(options.AnnounceInterval);
        }

        public FleetCommandServer(NetworkLink link, Timekeeper timekeeper) : this(link, timekeeper, new ServerOptions())
        { }

        public void UpdateVessels()
        {
            int now = _timekeeper.Now;

            foreach (long key in _knownVessels.Keys.ToArray())
            {
                VesselData data = _knownVessels[key];
                if (data.Id == _self.Id)
                {
                    data.LastSeen = now;
                    continue;
                }
                
                if ((now - data.LastSeen) >= _vesselTimeoutTicks)
                {
                    _knownVessels.Remove(key);
                    OnVesselLost?.Invoke(data);
                }
            }

            if (now % _announceIntervalTicks == 0)
            {
                VesselBeacon beacon = new VesselBeacon(_self.Id, _self.NetworkId);
                _link.SendPublicBroadcast((ushort)SystemNetMessage.AnnounceVessel, beacon);
            }
        }

        public void UpdateNetworks()
        {
            int now = _timekeeper.Now;
            foreach (long key in _knownNetworks.Keys.ToArray())
            {
                NetworkData data = _knownNetworks[key];
                if (data.Id == _self.NetworkId)
                {
                    data.LastSeen = now;
                    continue;
                }

                if ((now - data.LastSeen) >= _networkTimeoutTicks)
                {
                    foreach (VesselData vessel in _knownVessels.Values.Where(v => v.NetworkId == data.Id && v.Id != _self.Id))
                        vessel.Verified = false; // membership cannot be trusted anymore

                    _knownNetworks.Remove(data.Id);
                    OnNetworkLost?.Invoke(data);
                }

                if (_self.NetworkId.HasValue && _selfNet.OwnerId == _self.Id && now % _announceIntervalTicks == 0)
                {
                    NetworkBeacon beacon = new NetworkBeacon(_selfNet.Id, _selfNet.OwnerId, _selfNet.Members);
                    _link.SendPublicBroadcast((ushort)SystemNetMessage.AnnounceNetwork, beacon);
                }
            }
        }

        public void Update()
        {
            UpdateVessels();
            UpdateNetworks();
        }
    }
}
