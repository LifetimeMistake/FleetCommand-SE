using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class Networks : IEnumerable<Network>
    {
        private Dictionary<long, Network> _networks;
        private Vessel _localVessel;

        public Networks(Vessel localVessel)
        {
            _localVessel = localVessel;
            _networks = new Dictionary<long, Network>();
        }

        public Network GetLocalNetwork()
        {
            if (!_localVessel.HasNetwork)
                return null;

            long netId = _localVessel.NetworkId.Value;
            if (!_networks.ContainsKey(netId))
                return null;

            return _networks[netId];
        }

        public Network Get(long id)
        {
            return _networks[id];
        }

        public Network GetAuthenticated(Vessel vessel)
        {
            long id = vessel.NetworkId.Value;
            if (!_networks.ContainsKey(id))
                return null;

            Network network = _networks[id];
            if (!network.Members.Contains(vessel.Id))
                return null;

            return network;
        }

        public IEnumerator<Network> GetEnumerator()
        {
            return _networks.Values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public bool Contains(long id)
        {
            return _networks.ContainsKey(id);
        }

        public void Add(Network network)
        {
            _networks.Add(network.Id, network);
        }

        public bool Remove(long id)
        {
            if (id == _localVessel.NetworkId)
                return false;

            return _networks.Remove(id);
        }
    }
}
