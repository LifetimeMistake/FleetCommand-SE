using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;

namespace FleetCommand.Protocol.Discovery
{
    public class VesselDiscovery
    {
        // optimizations to reduce heap allocations during ticks
        private List<Vessel> _dropped = new List<Vessel>();
        private NetworkLink _link;
        private Vessel _localVessel;
        private Vessels _vessels;
        private Networks _networks;
        private Timekeeper _timekeeper;

        private int _vesselTimeout;
        private int _announceInterval;
        private int _lastAnnounce;

        public delegate void VesselDiscoveryDelegate(Vessel vessel);
        public event VesselDiscoveryDelegate OnVesselDiscovered;
        public event VesselDiscoveryDelegate OnVesselLost;

        public VesselDiscovery(NetworkLink link, Vessels vessels, Networks networks, Timekeeper timekeeper, float vesselTimeout, float announceInterval)
        {
            _link = link;
            _vessels = vessels;
            _networks = networks;
            _localVessel = vessels.GetLocalVessel();
            _timekeeper = timekeeper;
            _vesselTimeout = timekeeper.SecondsToTicks(vesselTimeout);
            _announceInterval = timekeeper.SecondsToTicks(announceInterval);
            _lastAnnounce = 0;

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.AnnounceVessel, ReceiveAnnounceVessel, NetMessageHandlerOptions.CreatePublicHandler(true, false)))
                throw new Exception("Could not register a message handler for AnnounceVessel");
        }

        private void ReceiveAnnounceVessel(NetInvocationContext context, BinaryReader reader)
        {
            NetMessageHeader header = context.Metadata;
            if (!header.HasData)
                return; // TODO: log error

            VesselBeacon beacon = VesselBeacon.Deserialize(reader);
            if (beacon.VesselId == _localVessel.Id)
                return; // do not update local vessel here

            if (!_vessels.Contains(beacon.VesselId))
            {
                // Vessel just got discovered
                Vessel vessel = new Vessel(beacon.VesselId, _timekeeper.Now);
                if (beacon.NetworkId.HasValue)
                {
                    // Find network data
                    vessel.NetworkId = beacon.NetworkId;
                    vessel.NetworkData = _networks.GetAuthenticated(vessel);
                }

                _vessels.Add(vessel);
                OnVesselDiscovered?.Invoke(vessel);
            }
            else
            {
                // Vessel is known already
                Vessel vessel = _vessels.Get(beacon.VesselId);
                if (vessel.NetworkId != beacon.NetworkId)
                {
                    vessel.NetworkId = beacon.NetworkId;
                    vessel.NetworkData = null;

                    // Do not update own network, we do not trust self-proclaimed network joins
                    if (beacon.NetworkId.HasValue && beacon.NetworkId != _localVessel.NetworkId)
                    {
                        vessel.NetworkData = _networks.GetAuthenticated(vessel);
                    }
                }
                
                vessel.LastSeen = _timekeeper.Now;
            }
        }

        public void Update()
        {
            int now = _timekeeper.Now;
            
            foreach (Vessel vessel in _vessels)
            {
                if (vessel.Id == _localVessel.Id)
                {
                    vessel.LastSeen = now;
                    continue;
                }

                if ((now - vessel.LastSeen) >= _vesselTimeout)
                {
                    _dropped.Add(vessel);
                }
            }

            if (_dropped.Count > 0)
            {
                foreach(Vessel vessel in _dropped)
                {
                    _vessels.Remove(vessel.Id);
                    OnVesselLost?.Invoke(vessel);
                }

                _dropped.Clear();
            }

            if ((now - _lastAnnounce) >= _announceInterval)
            {
                VesselBeacon beacon = new VesselBeacon(_localVessel.Id, _localVessel.NetworkId);
                _link.SendPublicBroadcast((ushort)SystemNetMessage.AnnounceVessel, beacon);
                _lastAnnounce = now;
            }
        }
    }
}
