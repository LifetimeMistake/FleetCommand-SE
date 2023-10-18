using FleetCommand.IO;
using FleetCommand.Protocol;
using FleetCommand.Protocol.Discovery;
using FleetCommand.Cryptography;
using FleetCommand.Networking;
using Sandbox.ModAPI.Ingame;
using System;
using System.Text;
using FleetCommand.Protocol.Ownership;

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
        private OwnerNegotiation _ownerNegotiation;
        private NetworkMembership _networkMembership;
        private string netLog;

        public Program()
        {
            // Initialize network link
            Stream readBuffer = new MemoryStream();
            Stream writeBuffer = new MemoryStream();
            bool leaveOpen = false;
            string senderTag = "FleetCommand";
            long? networkId = null;
            _networkLink = new NetworkLink(readBuffer, writeBuffer, leaveOpen, senderTag, IGC, networkId);
            NetworkLink.Log = new Action<string>((msg) => netLog = msg);

            // Initialize vessels
            Vessel localVessel = new Vessel(IGC.Me, null, 0);
            _vessels = new Vessels(localVessel);

            // Initialize networks
            _networks = new Networks(_vessels.GetLocalVessel());

            // Initialize timekeeper
            int ticksPerSecond = 60;
            int ticksPerUpdate = 1;
            _timekeeper = new Timekeeper(ticksPerSecond, ticksPerUpdate);

            // Initialize vessel discovery
            float vesselTimeout = 2f;
            float announceInterval = 1f;
            _vesselDiscovery = new VesselDiscovery(_networkLink, _vessels, _networks, _timekeeper, vesselTimeout, announceInterval);

            // Initialize network discovery
            float networkTimeout = 5f;
            _networkDiscovery = new NetworkDiscovery(_networkLink, _vessels, _networks, _timekeeper, networkTimeout, announceInterval);

            // Initialize owner negotiation
            float ownerTimeout = 4f;
            _ownerNegotiation = new OwnerNegotiation(_networkLink, _timekeeper, _vessels, _networks, ownerTimeout);

            // Initialize network membership
            float joinTimeout = 1f;
            _networkMembership = new NetworkMembership(_networkLink, _timekeeper, _vessels, _networks, joinTimeout);

            Runtime.UpdateFrequency = UpdateFrequency.Update1;
        }

        public void Main(string argument, UpdateType updateSource)
        {
            if (updateSource == UpdateType.Update1)
            {
                netLog = null;
                // Process network packets
                _networkLink.ProcessPackets();
                // Update time
                _timekeeper.Update();

                // Update vessel and network discovery
                _vesselDiscovery.Update();
                _networkDiscovery.Update();
                // Update owner negotiation
                _ownerNegotiation.Update();
                // Update network membership
                _networkMembership.Update();

                StringBuilder sb = new StringBuilder();
                if (netLog != null)
                    sb.AppendLine($"NET: {netLog}");
                sb.AppendLine("Known vessels:");
                foreach (Vessel v in _vessels)
                {
                    sb.AppendLine($"{v.Id}/{(v.HasNetwork ? v.NetworkId.Value.ToString() : "None")} ({(v.Id == _vessels.GetLocalVessel().Id ? "Self" : "Foreign")})");
                }

                sb.AppendLine("Known networks:");
                foreach (Network n in _networks)
                {
                    sb.AppendLine($"{n.Id}/{(n.OwnerId.HasValue ? n.OwnerId.Value.ToString() : "None")}/M:{n.MemberCount}");
                }

                Echo(sb.ToString());
            }
            else
            {
                string[] commands = argument.Split(' ');
                if (commands.Length >= 2)
                {
                    string command = commands[0];
                    long arg;
                    if (long.TryParse(commands[1], out arg))
                    {
                        if (command == "join")
                        {
                            _networkMembership.Join(arg);
                        }
                        else if (command == "kick")
                        {
                            _networkMembership.Kick(arg);
                        }
                    }
                }
                else if (commands.Length == 1 && commands[0] == "create")
                {
                    _networkMembership.Create();
                }
                else if (commands.Length == 1 && commands[0] == "leave")
                {
                    _networkMembership.Leave();
                }
            }
        }
    }
}