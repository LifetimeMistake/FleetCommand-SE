using FleetCommand.IO;
using FleetCommand.Networking;
using System;
using System.Collections.Generic;
using System.Text;

namespace FleetCommand.Protocol.Membership
{
    public struct MemberUpdateData : ISerializable
    {
        public long MemberId;

        public MemberUpdateData(long memberId)
        {
            MemberId = memberId;
        }

        public void Serialize(BinaryWriter writer)
        {
            writer.Write(MemberId);
        }

        public static MemberUpdateData Deserialize(BinaryReader reader)
        {
            return new MemberUpdateData(reader.ReadInt64());
        }
    }
}
