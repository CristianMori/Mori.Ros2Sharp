using System.Net;
using Mori.Ros2Sharp;

namespace DdsProbe;

/// <summary>Offline round-trip checks for the CDR, parameter-list, SPDP, and RTPS framing layers.</summary>
internal static class SelfTest
{
    private static int _failures;

    public static int Run()
    {
        // CDR primitives and alignment: a double after a single byte pads to offset 8.
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.Write((byte)7);
        w.Write(3.25);
        byte[] cdr = w.ToArray();
        Check("cdr encapsulation header", cdr[0] == 0x00 && cdr[1] == 0x01);
        Check("cdr 8-byte alignment", cdr.Length == 4 + 16);
        var r = new CdrReader(cdr);
        Check("cdr byte round-trip", r.ReadUInt8() == 7);
        Check("cdr double round-trip", r.ReadFloat64() == 3.25);
        Check("cdr fully consumed", r.Remaining == 0);

        // Strings carry a NUL inside their length; following values re-align.
        var w2 = new CdrWriter(CdrEncapsulation.CdrLe);
        w2.Write("chatter");
        w2.Write((short)-2);
        var r2 = new CdrReader(w2.ToArray());
        Check("cdr string round-trip", r2.ReadString() == "chatter" && r2.ReadInt16() == -2);

        // SPDP participant data through PL_CDR_LE and back.
        var pd = new ParticipantData
        {
            Guid = new RtpsGuid(GuidPrefix.NewUnique(), EntityId.Participant),
            BuiltinEndpoints = BuiltinEndpoints.ParticipantAnnouncer | BuiltinEndpoints.ParticipantDetector,
            DomainId = 7,
            EntityName = "probe",
            LeaseDuration = RtpsDuration.FromSeconds(12),
        };
        pd.MetatrafficUnicastLocators.Add(Locator.UdpV4(IPAddress.Parse("192.168.1.10"), 7412));
        pd.DefaultUnicastLocators.Add(Locator.UdpV4(IPAddress.Parse("192.168.1.10"), 7413));
        byte[] payload = pd.Encode();
        Check("spdp payload is PL_CDR_LE", payload[0] == 0x00 && payload[1] == 0x03);
        Check("spdp decode", ParticipantData.TryDecode(payload, out var back) && back != null);
        Check("spdp guid round-trip", back!.Guid == pd.Guid);
        Check("spdp endpoints round-trip", back.BuiltinEndpoints == pd.BuiltinEndpoints);
        Check("spdp locator round-trip",
            back.MetatrafficUnicastLocators.Count == 1 &&
            back.MetatrafficUnicastLocators[0].Port == 7412 &&
            back.MetatrafficUnicastLocators[0].Address.Equals(IPAddress.Parse("192.168.1.10")) &&
            back.DefaultUnicastLocators.Count == 1);
        Check("spdp name/domain/lease round-trip",
            back.EntityName == "probe" && back.DomainId == 7 && back.LeaseDuration.Seconds == 12);

        // A full datagram: header + INFO_TS + DATA(p), parsed back.
        var mw = new RtpsMessageWriter(pd.Guid.Prefix);
        mw.AddInfoTimestamp(new RtpsTime(1_700_000_000, 42));
        mw.AddData(EntityId.SpdpReader, EntityId.SpdpWriter, 3, payload);
        byte[] datagram = mw.ToArray();
        Check("rtps magic", datagram[0] == 'R' && datagram[3] == 'S');
        Check("rtps parse", RtpsMessage.TryParse(datagram, out var msg) && msg != null);
        Check("rtps source prefix", msg!.Source == pd.Guid.Prefix);
        Check("rtps data submessage",
            msg.Data.Count == 1 &&
            msg.Data[0].WriterId == EntityId.SpdpWriter &&
            msg.Data[0].ReaderId == EntityId.SpdpReader &&
            msg.Data[0].SequenceNumber == 3);
        Check("rtps timestamp carried", msg.Data[0].Timestamp is { } t && t.Seconds == 1_700_000_000 && t.Fraction == 42);
        Check("rtps payload intact", msg.Data[0].Payload.AsSpan().SequenceEqual(payload));
        Check("spdp re-decodes from datagram", ParticipantData.TryDecode(msg.Data[0].Payload, out _));

        // SEDP endpoint data through PL_CDR_LE and back.
        var ed = new EndpointData
        {
            Guid = new RtpsGuid(pd.Guid.Prefix, new EntityId(0x00000103)),
            TopicName = Ros2Names.Topic("/chatter"),
            TypeName = Ros2Names.Type("std_msgs/msg/String"),
            Reliable = true,
        };
        ed.UnicastLocators.Add(Locator.UdpV4(IPAddress.Parse("192.168.1.10"), 7413));
        Check("ros2 name mangling",
            ed.TopicName == "rt/chatter" && ed.TypeName == "std_msgs::msg::dds_::String_");
        Check("sedp decode", EndpointData.TryDecode(ed.Encode(), out var edBack) && edBack != null);
        Check("sedp round-trip",
            edBack!.Guid == ed.Guid && edBack.TopicName == ed.TopicName && edBack.TypeName == ed.TypeName &&
            edBack.Reliable && !edBack.TransientLocal && edBack.UnicastLocators.Count == 1);

        // Reliability submessages: INFO_DST + HEARTBEAT + ACKNACK + GAP through the framing.
        var mw2 = new RtpsMessageWriter(pd.Guid.Prefix);
        mw2.AddInfoDestination(ed.Guid.Prefix);
        mw2.AddHeartbeat(EntityId.SedpPublicationsReader, EntityId.SedpPublicationsWriter, 2, 9, 5, final: false);
        mw2.AddAckNack(EntityId.SedpPublicationsReader, EntityId.SedpPublicationsWriter, 3,
            new long[] { 3, 5, 36 }, 7, final: false);
        mw2.AddGap(EntityId.SedpPublicationsReader, EntityId.SedpPublicationsWriter, 1, 3);
        Check("reliability parse", RtpsMessage.TryParse(mw2.ToArray(), out var msg2) && msg2 != null);
        Check("info_dst carried", msg2!.Destination == ed.Guid.Prefix);
        Check("heartbeat round-trip",
            msg2.Heartbeats.Count == 1 && msg2.Heartbeats[0].FirstSn == 2 &&
            msg2.Heartbeats[0].LastSn == 9 && msg2.Heartbeats[0].Count == 5 && !msg2.Heartbeats[0].Final);
        Check("acknack round-trip",
            msg2.AckNacks.Count == 1 && msg2.AckNacks[0].BaseSn == 3 &&
            msg2.AckNacks[0].Missing.SequenceEqual(new long[] { 3, 5, 36 }) && msg2.AckNacks[0].Count == 7);
        Check("gap round-trip",
            msg2.Gaps.Count == 1 && msg2.Gaps[0].GapStart == 1 && msg2.Gaps[0].GapListBase == 3);

        // A dispose DATA: no payload, key hash + status in inline QoS.
        var mw3 = new RtpsMessageWriter(pd.Guid.Prefix);
        mw3.AddDisposeData(EntityId.SpdpReader, EntityId.SpdpWriter, 4, pd.Guid);
        Span<byte> keyBytes = stackalloc byte[16];
        pd.Guid.WriteTo(keyBytes);
        Check("dispose parse", RtpsMessage.TryParse(mw3.ToArray(), out var msg3) && msg3!.Data.Count == 1);
        Check("dispose round-trip",
            msg3!.Data[0].Disposed && msg3.Data[0].Payload.Length == 0 &&
            msg3.Data[0].KeyHash != null && msg3.Data[0].KeyHash.AsSpan().SequenceEqual(keyBytes));

        // Service naming rules.
        Check("service name mangling",
            Ros2Names.ServiceRequestTopic("/add_two_ints") == "rq/add_two_intsRequest" &&
            Ros2Names.ServiceReplyTopic("/add_two_ints") == "rr/add_two_intsReply" &&
            Ros2Names.ServiceRequestType("example_interfaces/srv/AddTwoInts") == "example_interfaces::srv::dds_::AddTwoInts_Request_" &&
            Ros2Names.ServiceReplyType("example_interfaces/srv/AddTwoInts") == "example_interfaces::srv::dds_::AddTwoInts_Response_");

        // Related sample identity through inline QoS and back.
        var mw4 = new RtpsMessageWriter(pd.Guid.Prefix);
        var requester = new RtpsGuid(GuidPrefix.NewUnique(), new EntityId(0x00000203));
        mw4.AddData(new EntityId(0x00000204), new EntityId(0x00000103), 9, payload, requester, 5);
        Check("related identity parse", RtpsMessage.TryParse(mw4.ToArray(), out var msg4) && msg4!.Data.Count == 1);
        Check("related identity round-trip",
            msg4!.Data[0].RelatedGuid == requester && msg4.Data[0].RelatedSn == 5 &&
            msg4.Data[0].SequenceNumber == 9 && msg4.Data[0].Payload.AsSpan().SequenceEqual(payload));

        // Fragment submessages through the framing. DATA_FRAG flags differ from DATA: 0x01
        // endian, 0x02 inline QoS, and no data-present bit.
        var fragReader = new EntityId(0x00000204);
        var fragWriter = new EntityId(0x00000103);
        byte[] big = new byte[10_000];
        new Random(1).NextBytes(big);
        var mw5 = new RtpsMessageWriter(pd.Guid.Prefix);
        mw5.AddDataFrag(fragReader, fragWriter, 11, fragmentStartingNum: 3, fragmentsInSubmessage: 2,
            fragmentSize: 3000, sampleSize: 10_000, big.AsSpan(6000, 4000), requester, 5);
        mw5.AddHeartbeatFrag(fragReader, fragWriter, 11, 4, 9);
        mw5.AddNackFrag(fragReader, fragWriter, 11, 2, new uint[] { 2, 4, 300 }, 6);
        byte[] dg5 = mw5.ToArray();
        Check("data_frag id and flags", dg5[20] == 0x16 && dg5[21] == 0x03);
        Check("data_frag parse", RtpsMessage.TryParse(dg5, out var msg5) && msg5!.DataFrags.Count == 1);
        var df = msg5!.DataFrags[0];
        Check("data_frag header round-trip",
            df.ReaderId == fragReader && df.WriterId == fragWriter && df.SequenceNumber == 11 &&
            df.FragmentStartingNum == 3 && df.FragmentsInSubmessage == 2 && df.FragmentSize == 3000 &&
            df.SampleSize == 10_000);
        Check("data_frag bytes trimmed to sample end",
            df.Fragments.Length == 4000 && df.Fragments.AsSpan().SequenceEqual(big.AsSpan(6000, 4000)));
        Check("data_frag inline qos", df.RelatedGuid == requester && df.RelatedSn == 5);
        Check("heartbeat_frag round-trip",
            msg5.HeartbeatFrags.Count == 1 && msg5.HeartbeatFrags[0].SequenceNumber == 11 &&
            msg5.HeartbeatFrags[0].LastFragmentNum == 4 && msg5.HeartbeatFrags[0].Count == 9);
        Check("nack_frag round-trip (256-bit window)",
            msg5.NackFrags.Count == 1 && msg5.NackFrags[0].SequenceNumber == 11 &&
            msg5.NackFrags[0].Missing.SequenceEqual(new uint[] { 2, 4 }) && msg5.NackFrags[0].Count == 6);

        // Reassembly: fragments out of order, duplicated, and interleaved between two samples;
        // the second sample has an odd size so its last fragment is short and unaligned.
        var fragWriterGuid = new RtpsGuid(pd.Guid.Prefix, fragWriter);
        var reader = new RtpsReaderEndpoint(null!, new RtpsGuid(GuidPrefix.NewUnique(), fragReader), "t", "T", reliable: true);
        reader.MatchWriter(fragWriterGuid, Array.Empty<IPEndPoint>());
        var delivered = new List<(long Sn, byte[] Payload)>();
        reader.SampleReceived += (_, d) => delivered.Add((d.SequenceNumber, d.Payload));
        byte[] odd = new byte[7_777];
        new Random(2).NextBytes(odd);
        RtpsDataFrag Frag(long sn, byte[] sample, uint start, ushort count)
        {
            var fw = new RtpsMessageWriter(pd.Guid.Prefix);
            int off = (int)(start - 1) * 3000;
            int len = Math.Min(count * 3000, sample.Length - off);
            fw.AddDataFrag(fragReader, fragWriter, sn, start, count, 3000, (uint)sample.Length, sample.AsSpan(off, len));
            RtpsMessage.TryParse(fw.ToArray(), out var fm);
            return fm!.DataFrags[0];
        }
        Check("frag: unmatched writer ignored", !reader.OnDataFrag(GuidPrefix.NewUnique(), Frag(1, big, 1, 1)));
        reader.OnDataFrag(pd.Guid.Prefix, Frag(1, big, 4, 1));  // last fragment first
        reader.OnDataFrag(pd.Guid.Prefix, Frag(2, odd, 1, 2));  // second sample interleaved
        reader.OnDataFrag(pd.Guid.Prefix, Frag(1, big, 2, 2));
        reader.OnDataFrag(pd.Guid.Prefix, Frag(1, big, 4, 1));  // duplicate
        Check("frag: incomplete samples held back", delivered.Count == 0);
        reader.OnDataFrag(pd.Guid.Prefix, Frag(1, big, 1, 1));
        Check("frag: sample reassembled out of order",
            delivered.Count == 1 && delivered[0].Sn == 1 && delivered[0].Payload.AsSpan().SequenceEqual(big));
        reader.OnDataFrag(pd.Guid.Prefix, Frag(2, odd, 3, 1));
        Check("frag: odd-sized sample reassembled",
            delivered.Count == 2 && delivered[1].Sn == 2 && delivered[1].Payload.AsSpan().SequenceEqual(odd));
        reader.OnDataFrag(pd.Guid.Prefix, Frag(1, big, 1, 4));
        Check("frag: completed sample not delivered twice", delivered.Count == 2);

        // rmw_dds_common/ParticipantEntitiesInfo layout (Humble: gids are char[24]).
        var pguid = new RtpsGuid(pd.Guid.Prefix, EntityId.Participant);
        byte[] pei = Ros2Node.EncodeParticipantEntitiesInfo(pguid, "/", "probe",
            new[] { new RtpsGuid(pd.Guid.Prefix, new EntityId(0x00000104)) },
            new[] { new RtpsGuid(pd.Guid.Prefix, new EntityId(0x00000103)) });
        var pr = new CdrReader(pei);
        Span<byte> gid = stackalloc byte[24];
        pguid.WriteTo(gid);
        Check("pei participant gid", pr.ReadBytes(24).AsSpan().SequenceEqual(gid));
        Check("pei node entry", pr.ReadLength() == 1 && pr.ReadString() == "/" && pr.ReadString() == "probe");
        Check("pei gid lists", pr.ReadLength() == 1 && pr.ReadBytes(24).Length == 24 &&
            pr.ReadLength() == 1 && pr.ReadBytes(24).Length == 24 && pr.Remaining == 0);

        Console.WriteLine(_failures == 0 ? "all checks passed" : $"{_failures} check(s) FAILED");
        return _failures == 0 ? 0 : 1;
    }

    private static void Check(string name, bool ok)
    {
        Console.WriteLine($"{(ok ? "ok  " : "FAIL")}  {name}");
        if (!ok) _failures++;
    }
}
