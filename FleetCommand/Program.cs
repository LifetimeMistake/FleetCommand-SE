using FleetCommand.IO;
using FleetCommand.Protocol;
using FleetCommand.Protocol.Discovery;
using FleetCommand.Cryptography;
using FleetCommand.Networking;
using Sandbox.ModAPI.Ingame;
using System;
using System.Text;

namespace FleetCommand
{
    partial class Program : MyGridProgram
    {
        private NetworkLink _networkLink;
        private Vessels _vessels;
        private Networks _networks;
        private Timekeeper _timekeeper;
        private VesselDiscovery _vesselDiscovery;
        private NetworkDiscovery _networkDiscovery;

        public Program()
        {
            // Initialize network link
            Stream readBuffer = new MemoryStream();
            Stream writeBuffer = new MemoryStream();
            bool leaveOpen = false;
            string senderTag = "FleetCommand";
            long? networkId = null;
            _networkLink = new NetworkLink(readBuffer, writeBuffer, leaveOpen, senderTag, IGC, networkId);

            // Initialize vessels
            Vessel localVessel = new Vessel(IGC.Me, 0); // Replace with your actual local vessel ID and last seen time
            _vessels = new Vessels(localVessel);

            // Initialize networks
            _networks = new Networks(_vessels.GetLocalVessel());

            // Initialize timekeeper
            int ticksPerSecond = 60; // Replace with your desired ticks per second
            int ticksPerUpdate = 1; // Replace with your desired ticks per update
            _timekeeper = new Timekeeper(ticksPerSecond, ticksPerUpdate);

            // Initialize vessel discovery
            float vesselTimeout = 2f; // Replace with your desired vessel timeout in seconds
            float announceInterval = 1f; // Replace with your desired announce interval in seconds
            _vesselDiscovery = new VesselDiscovery(_networkLink, _vessels, _networks, _timekeeper, vesselTimeout, announceInterval);

            // Initialize network discovery
            float networkTimeout = 5f; // Replace with your desired network timeout in seconds
            _networkDiscovery = new NetworkDiscovery(_networkLink, _vessels, _networks, _timekeeper, networkTimeout, announceInterval);

            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            // Process network packets
            _networkLink.ProcessPackets();
            // Update time
            _timekeeper.Update();

            // Update vessel and network discovery
            _vesselDiscovery.Update();
            _networkDiscovery.Update();

            StringBuilder sb = new StringBuilder();
            sb.AppendLine("Known vessels:");
            foreach(Vessel v in _vessels)
            {
                sb.AppendLine($"{v.Id}/{(v.HasNetwork ? v.NetworkId.Value.ToString() : "None")} ({(v.Id == _vessels.GetLocalVessel().Id ? "Self" : "Foreign")})");
            }

            Echo(sb.ToString());
        }
    }
}
