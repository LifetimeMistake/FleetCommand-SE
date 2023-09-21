using FleetCommand.Networking.Math;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace FleetCommand.Networking
{
    public class NetMessageRegistry
    {
        private Dictionary<string, Range<ushort>> reservedMessageTypes;

        public int ReservationCount { get { return reservedMessageTypes.Count; } }
        public int ReservedTypeCount { get { return reservedMessageTypes.Values.Sum(r => r.To - r.From + 1);  } }

        public NetMessageRegistry()
        {
            reservedMessageTypes = new Dictionary<string, Range<ushort>>();
        }
        
        public bool ReserveTypes(string tag, ushort from, ushort to)
        {
            return ReserveTypes(tag, new Range<ushort>(from, to));
        }

        public bool ReserveTypes(string tag, Range<ushort> range)
        {
            if (reservedMessageTypes.ContainsKey(tag))
                return false;

            if (reservedMessageTypes.Values.Any(r => r.Intersects(range))) 
                return false;

            reservedMessageTypes.Add(tag, range);
            return true;
        }

        public bool RemoveReservation(string tag)
        {
            return reservedMessageTypes.Remove(tag);
        }

        public bool IsReserved(ushort type)
        {
            return reservedMessageTypes.Values.Any(r => r.Contains(type));
        }

        public bool IsReserved(Range<ushort> types)
        {
            return reservedMessageTypes.Values.Any(r => r.Intersects(types));
        }

        public bool IsReserved(string tag)
        {
            return reservedMessageTypes.ContainsKey(tag);
        }

        public string GetCollisionSource(ushort type)
        {
            foreach(KeyValuePair<string, Range<ushort>> kvp in reservedMessageTypes)
                if (kvp.Value.Contains(type))
                    return kvp.Key;

            return null;
        }

        public string GetCollisionSource(Range<ushort> types)
        {
            foreach (KeyValuePair<string, Range<ushort>> kvp in reservedMessageTypes)
                if (kvp.Value.Intersects(types))
                    return kvp.Key;

            return null;
        }
    }
}
