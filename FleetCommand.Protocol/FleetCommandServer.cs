using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class FleetCommandServer
    {
        private NetworkLink _link;
        private ServerOptions _config;
        private VesselData _self;
        private DateTime _lastVesselAnnounce;
        private NetworkData _selfNet;
        private DateTime _lastNetworkAnnounce;
        private Dictionary<long, VesselData> _knownVessels;
        private Dictionary<long, NetworkData> _knownNetworks;

        public delegate void VesselDelegate(VesselData vessel);
        public delegate void NetworkDiscoveryDelegate(NetworkData net);

        public event VesselDelegate OnVesselDiscovered;
        public event VesselDelegate OnVesselLost;
        public event NetworkDiscoveryDelegate OnNetworkDiscovered;
        public event NetworkDiscoveryDelegate OnNetworkLost;

        public FleetCommandServer(NetworkLink link, ServerOptions options)
        {
            _link = link;
            _config = options;
            _knownVessels = new Dictionary<long, VesselData>();
            _knownNetworks = new Dictionary<long, NetworkData>();

            _self = new VesselData(link.OwnId, link.OwnNetworkId);
            _self.Verified = true;
            _knownVessels.Add(link.OwnId, _self);

            if (link.OwnNetworkId.HasValue)
            {
                _selfNet = new NetworkData(link.OwnNetworkId.Value, null);
                _knownNetworks.Add(_selfNet.Id, _selfNet);
            }
        }

        public FleetCommandServer(NetworkLink link) : this(link, new ServerOptions())
        { }

        public void UpdateVessels()
        {
            DateTime now = DateTime.Now;
            foreach (long key in _knownVessels.Keys.ToArray())
            {
                VesselData data = _knownVessels[key];
                if (data.Id == _self.Id)
                {
                    data.LastSeen = DateTime.Now;
                    continue;
                }
                
                if ((now - data.LastSeen).TotalSeconds >= _config.VesselTimeout)
                {
                    _knownVessels.Remove(key);
                    OnVesselLost?.Invoke(data);
                }
            }
        }

        public void UpdateNetworks()
        {
            DateTime now = DateTime.Now;
            foreach (long key in _knownNetworks.Keys.ToArray())
            {
                NetworkData data = _knownNetworks[key];
                if (data.Id == _self.NetworkId)
                {
                    data.LastSeen = DateTime.Now;
                    continue;
                }

                if ((now - data.LastSeen).TotalSeconds >= _config.NetworkTimeout)
                {
                    foreach (VesselData vessel in _knownVessels.Where(v => v.NetworkId == data.Id && v.Id != _self.Id))
                        vessel.Verified = false; // membership cannot be trusted anymore

                    _knownNetworks.Remove(data.Id);
                    OnNetworkLost?.Invoke(data);
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
