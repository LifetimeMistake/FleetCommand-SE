using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace FleetCommand.Protocol.Ownership
{
    public class OwnerNegotiation
    {
        private NetworkLink _link;
        private Timekeeper _timekeeper;
        private Vessel _localVessel;
        private Vessels _vessels;
        private int _ownerTimeout;

        public delegate void OwnerUpdateDelegate(long ownerId);
        public event OwnerUpdateDelegate OnOwnerUpdated;

        public OwnerNegotiation(NetworkLink link, Timekeeper timekeeper, Vessels vessels, float ownerTimeout)
        {
            _link = link;
            _timekeeper = timekeeper;
            _vessels = vessels;
            _localVessel = _vessels.GetLocalVessel();
            _ownerTimeout = _timekeeper.SecondsToTicks(ownerTimeout);

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.ElevateOwner, ReceiveElevateOwner, NetMessageHandlerOptions.CreateOwnHandler(true, false)))
                throw new Exception("Could not register a message handler for ElevateOwner");
        }

        private void ReceiveElevateOwner(NetInvocationContext context, BinaryReader reader)
        {
            if (!_localVessel.NetworkDataAvailable || _localVessel.OwnsNetwork)
                return;

            Network network = _localVessel.NetworkData;
            long nextOwner;

            if (network.HasOwner)
            {
                Vessel owner = _vessels.Get(network.OwnerId.Value);
                if (owner != null && (_timekeeper.Now - owner.LastSeen) < _ownerTimeout)
                    return;

                nextOwner = network.Members.Where(t => t != network.OwnerId.Value).OrderBy(t => t).First();
            }
            else
            {
                nextOwner = network.Members.OrderBy(t => t).First();
            }

            if (context.Metadata.SourceId != nextOwner)
                return;

            network.OwnerId = nextOwner;
            OnOwnerUpdated?.Invoke(nextOwner);
        }

        public void Update()
        {
            if (!_localVessel.NetworkDataAvailable || _localVessel.OwnsNetwork)
                return;

            Network network = _localVessel.NetworkData;

            long nextOwner;
            if (network.HasOwner)
            {
                Vessel owner = _vessels.Get(network.OwnerId.Value);
                // Possible that the vessel has not been discovered yet
                if (owner == null || (_timekeeper.Now - owner.LastSeen) < _ownerTimeout)
                    return;

                nextOwner = network.Members.Where(t => t != owner.Id).OrderBy(t => t).First();
            }
            else
            {
                nextOwner = network.Members.OrderBy(t => t).First();
            }

            if (nextOwner == _localVessel.Id)
                _link.SendNetworkBroadcast((ushort)SystemNetMessage.ElevateOwner, null);

            network.OwnerId = nextOwner;
            OnOwnerUpdated?.Invoke(nextOwner);
        }
    }
}
