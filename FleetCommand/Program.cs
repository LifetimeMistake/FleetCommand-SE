using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using FleetCommand.Common;
using FleetCommand.Common.Logger;
using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        private TimeSpan _elapsed = new TimeSpan();
        private Logger _logger;

        public Program()
        {
            IMyTextPanel lcd = (IMyTextPanel)GridTerminalSystem.GetBlockWithName("Logger LCD");
            _logger = new Logger(30, lcd, Echo);
            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            _elapsed += Runtime.TimeSinceLastRun;
            TimeSource.UpdateTime(_elapsed);

            _logger.Info("testing", "TestService");
        }
    }
}
