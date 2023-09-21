using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace FleetCommand.Protocol
{
    public class VesselDiscovery
    {
        private Dictionary<long, VesselDiscoveryData> _vessels;
        private float _timeout;

        public delegate void VesselDelegate(long vesselId);

        public event VesselDelegate OnVesselDiscovered;
        public event VesselDelegate OnVesselLost;

        public VesselDiscovery(NetworkLink link, float vesselTimeout = 2f)
        {
            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.AnnounceVessel, ReceiveAnnounceVessel, NetMessageHandlerOptions.CreatePublicHandler(true, false)))
                throw new Exception("Could not register a message handler for AnnounceVessel");

            _vessels = new Dictionary<long, VesselDiscoveryData>();
            _timeout = vesselTimeout;
        }

        private void ReceiveAnnounceVessel(NetInvocationContext context, BinaryReader reader)
        {
            NetMessageHeader header = context.Metadata;
            if (!header.HasData)
                throw new Exception("Missing data");

            VesselBeacon beacon = VesselBeacon.Deserialize(reader);
            if (!_vessels.ContainsKey(beacon.VesselId))
            {
                // Vessel just got discovered
                _vessels.Add(beacon.VesselId, new VesselDiscoveryData(beacon.VesselId, DateTime.Now));
                OnVesselDiscovered?.Invoke(beacon.VesselId);
            }
            else
            {
                // Vessel is known already
                VesselDiscoveryData data = _vessels[beacon.VesselId];
                data.LastSeen = DateTime.Now;
            }
        }

        public void Update()
        {
            DateTime now = DateTime.Now;
            foreach (long key in _vessels.Keys.ToArray())
            {
                VesselDiscoveryData data = _vessels[key];
                if ((now - data.LastSeen).TotalSeconds >= _timeout)
                {
                    _vessels.Remove(key);
                    OnVesselLost?.Invoke(data.VesselId);
                }
            }
        }
    }
}
