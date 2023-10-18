using System;
using FleetCommand.IO;
using FleetCommand.Networking;
using FleetCommand.Protocol.Membership;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace FleetCommand.Protocol
{
    public class NetworkMembership
    {
        private NetworkLink _link;
        private Timekeeper _timekeeper;
        private Networks _networks;
        private Vessels _vessels;
        private Vessel _localVessel;
        private NetworkJoinInfo? _pendingJoin;
        private int _expirationTime;

        public delegate void MemberJoinDelegate(long vesselId);
        public delegate void MemberLeaveDelegate(long vesselId, LeaveReason reason);
        public delegate void NetworkJoinDelegate(long networkId);
        public delegate void NetworkJoinFailedDelegate(long networkId, JoinResult result);
        public delegate void NetworkLeaveDelegate(long networkId, LeaveReason reason);

        public event MemberJoinDelegate OnMemberJoined;
        public event MemberLeaveDelegate OnMemberLeft;
        public event NetworkJoinDelegate OnNetworkJoin;
        public event NetworkJoinFailedDelegate OnNetworkJoinFailed;
        public event NetworkLeaveDelegate OnNetworkLeave;

        public NetworkMembership(NetworkLink link, Timekeeper timekeeper, Vessels vessels, Networks networks, float joinTimeout)
        {
            _link = link;
            _timekeeper = timekeeper;
            _vessels = vessels;
            _networks = networks;
            _localVessel = _vessels.GetLocalVessel();
            _expirationTime = timekeeper.SecondsToTicks(joinTimeout);

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.JoinNetwork, ReceiveJoinNetwork, NetMessageHandlerOptions.CreatePublicHandler(false, true)))
                throw new Exception("Could not register a message handler for JoinNetwork");

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.NetworkJoinResponse, ReceiveNetworkJoinResponse, NetMessageHandlerOptions.CreatePublicHandler(false, true)))
                throw new Exception("Could not register a message handler for NetworkJoinResponse");

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.LeaveNetwork, ReceiveLeaveNetwork, NetMessageHandlerOptions.CreateOwnHandler(true, false)))
                throw new Exception("Could not register a message handler for LeaveNetwork");

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.MemberJoined, ReceiveMemberJoined, NetMessageHandlerOptions.CreateOwnHandler(true, false)))
                throw new Exception("Could not register a message handler for MemberJoined");

            if (!link.RegisterMessageHandler((ushort)SystemNetMessage.MemberKicked, ReceiveMemberKicked, NetMessageHandlerOptions.CreateOwnHandler(true, false)))
                throw new Exception("Could not register a message handler for MemberKicked");
        }

        private void ReceiveJoinNetwork(NetInvocationContext context, BinaryReader reader)
        {
            Network network = _networks.GetLocalNetwork();
            if (network == null || _localVessel.Id != network.OwnerId)
                return;

            long vesselId = context.Metadata.SourceId;
            JoinResult joinResult;
            if (network.Members.Contains(vesselId))
            {
                joinResult = JoinResult.AlreadyJoined;
            }
            else
            {
                joinResult = JoinResult.OK;
                network.Members.Add(vesselId);
                OnMemberJoined?.Invoke(vesselId);
            }

            JoinNetworkResponseData data = new JoinNetworkResponseData(network.Id, joinResult);
            _link.SendPublicUnicast(vesselId, (ushort)SystemNetMessage.NetworkJoinResponse, data);
        }

        private void ReceiveNetworkJoinResponse(NetInvocationContext context, BinaryReader reader)
        {
            if (_localVessel.HasNetwork || !_pendingJoin.HasValue)
                return;

            JoinNetworkResponseData response = JoinNetworkResponseData.Deserialize(reader);
            NetworkJoinInfo info = _pendingJoin.Value;
            if (info.NetworkId != response.NetworkId)
                return;

            Network network = _networks.Get(info.NetworkId);
            if (context.Metadata.SourceId != network.OwnerId)
                return;

            if (response.Result == JoinResult.OK)
            {
                network.Members.Add(_localVessel.Id);
                _localVessel.NetworkId = info.NetworkId;
                _link.OwnNetworkId = info.NetworkId;
                OnNetworkJoin?.Invoke(info.NetworkId);
            }
            else
            {
                OnNetworkJoinFailed?.Invoke(info.NetworkId, response.Result);
            }

            _pendingJoin = null;
        }

        private void ReceiveLeaveNetwork(NetInvocationContext context, BinaryReader reader)
        {
            if (!_localVessel.HasNetwork || !context.Metadata.HasData)
                return;

            Network network = _networks.GetLocalNetwork();
            MemberUpdateData data = MemberUpdateData.Deserialize(reader);
            if (!network.Members.Remove(data.MemberId))
                return;

            OnMemberLeft?.Invoke(data.MemberId, LeaveReason.Normal);
        }

        private void ReceiveMemberJoined(NetInvocationContext context, BinaryReader reader)
        {
            if (!_localVessel.HasNetwork || !context.Metadata.HasData)
                return;

            Network network = _networks.GetLocalNetwork();
            if (context.Metadata.SourceId != network.OwnerId)
                return;

            MemberUpdateData data = MemberUpdateData.Deserialize(reader);
            if (network.Members.Contains(data.MemberId))
                return;

            network.Members.Add(data.MemberId);
            OnMemberJoined?.Invoke(data.MemberId);
        }

        private void ReceiveMemberKicked(NetInvocationContext context, BinaryReader reader)
        {
            if (!_localVessel.HasNetwork || !context.Metadata.HasData)
                return;

            Network network = _networks.GetLocalNetwork();
            if (context.Metadata.SourceId != network.OwnerId)
                return;

            MemberUpdateData data = MemberUpdateData.Deserialize(reader);
            if (!network.Members.Remove(data.MemberId))
                return;

            OnMemberLeft?.Invoke(data.MemberId, LeaveReason.Kicked);
        }

        public void Update()
        {
            if (!_pendingJoin.HasValue || (_pendingJoin.Value.ExpirationTime < _timekeeper.Now))
                return;

            NetworkJoinInfo info = _pendingJoin.Value;
            _pendingJoin = null;
            OnNetworkJoinFailed?.Invoke(info.NetworkId, JoinResult.Timeout);
        }

        public bool Create()
        {
            if (_localVessel.HasNetwork)
                return false;

            long netId = _localVessel.Id + DateTime.Now.Ticks;
            Network network = new Network(netId, _localVessel.Id, _timekeeper.Now);
            _networks.Add(network);

            _localVessel.NetworkId = netId;
            return true;
        }

        public bool Join(long networkId)
        {
            if (_pendingJoin.HasValue || _localVessel.HasNetwork)
                return false;

            Network network = _networks.Get(networkId);
            if (network == null || !network.HasOwner)
                return false;

            NetworkJoinInfo info = new NetworkJoinInfo(networkId, _timekeeper.Now + _expirationTime);
            _pendingJoin = info;
            _link.SendPublicUnicast(network.OwnerId.Value, (ushort)SystemNetMessage.JoinNetwork, null);
            return true;
        }

        public bool Leave(LeaveReason reason = LeaveReason.Normal)
        {
            if (!_localVessel.HasNetwork)
                return false;

            long networkId = _localVessel.NetworkId.Value;
            _localVessel.NetworkId = null;
            _link.SendNetworkBroadcast((ushort)SystemNetMessage.LeaveNetwork, null);
            _link.OwnNetworkId = null;
            OnNetworkLeave?.Invoke(networkId, reason);
            return true;
        }

        public bool Kick(long vesselId)
        {
            Network network = _networks.GetLocalNetwork();
            if (network == null || _localVessel.Id != network.OwnerId)
                return false;

            if (!network.Members.Contains(vesselId))
                return false;

            _link.SendNetworkBroadcast((ushort)SystemNetMessage.MemberKicked, new MemberUpdateData(vesselId));
            network.Members.Remove(vesselId);
            return true;
        }
    }
}
