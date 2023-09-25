using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol
{
    public class Vessels : IEnumerable<Vessel>
    {
        private Dictionary<long, Vessel> _vessels;
        private Vessel _localVessel;

        public Vessels(Vessel localVessel)
        {
            _localVessel = localVessel;
            _vessels = new Dictionary<long, Vessel>
            {
                { localVessel.Id, localVessel }
            };
        }

        public Vessel GetLocalVessel()
        {
            return _localVessel;
        }

        public Vessel Get(long id)
        {
            return _vessels[id];
        }

        public IEnumerator<Vessel> GetEnumerator()
        {
            return _vessels.Values.GetEnumerator();
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public bool Contains(long id)
        {
            return _vessels.ContainsKey(id);
        }

        public void Add(Vessel vessel)
        {
            _vessels.Add(vessel.Id, vessel);
        }

        public bool Remove(long id)
        {
            if (id == _localVessel.Id)
                return false;

            return _vessels.Remove(id);
        }
    }
}
