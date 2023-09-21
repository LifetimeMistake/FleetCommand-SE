using System;
using System.Collections.Generic;
using System.Text;
using VRageRender.Messages;

namespace FleetCommand.Protocol
{
    public class Timekeeper
    {
        public int Now;

        private int _ticksPerSecond;
        private int _ticksPerUpdate;

        public Timekeeper(int ticksPerSecond, int ticksPerUpdate)
        {
            _ticksPerSecond = ticksPerSecond;
            _ticksPerUpdate = ticksPerUpdate;
            Now = 0;
        }

        public void Update()
        {
            Now += _ticksPerUpdate;
        }

        public float GetTotalSeconds()
        {
            return TicksToSeconds(Now);
        }

        public float TicksToSeconds(int ticks)
        {
            return (float)Math.Round((double)ticks / _ticksPerSecond);
        }

        public int SecondsToTicks(float seconds)
        {
            return (int)Math.Round(seconds * _ticksPerSecond);
        }
    }
}
