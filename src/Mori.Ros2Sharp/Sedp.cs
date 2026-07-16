using System.Buffers.Binary;

namespace Mori.Ros2Sharp;

/// <summary>
/// What SEDP says about one endpoint — the payload of DATA(w) (DiscoveredWriterData, carried by
/// the builtin publications writer) and DATA(r) (DiscoveredReaderData, subscriptions writer).
/// The two payloads share this format; which builtin topic carries them tells writer from reader.
/// </summary>
public sealed class EndpointData
{
    public RtpsGuid Guid { get; set; }
    public string TopicName { get; set; } = "";
    public string TypeName { get; set; } = "";
    public bool Reliable { get; set; }
    public bool TransientLocal { get; set; }
    public List<Locator> UnicastLocators { get; } = new();

    /// <summary>The PL_CDR_LE payload announcing one of our endpoints.</summary>
    public byte[] Encode()
    {
        var p = new ParameterListWriter();
        p.Add(Pid.EndpointGuid, Guid);
        p.Add(Pid.TopicName, TopicName);
        p.Add(Pid.TypeName, TypeName);

        // ReliabilityQosPolicy: kind (best-effort=1, reliable=2 on the wire) + max_blocking_time.
        Span<byte> rel = stackalloc byte[12];
        BinaryPrimitives.WriteUInt32LittleEndian(rel, Reliable ? 2u : 1u);
        BinaryPrimitives.WriteInt32LittleEndian(rel.Slice(4), 0);
        BinaryPrimitives.WriteUInt32LittleEndian(rel.Slice(8), 429496730); // 100 ms
        p.Add(Pid.Reliability, rel);

        p.Add(Pid.Durability, TransientLocal ? 1u : 0u);

        foreach (var l in UnicastLocators) p.Add(Pid.UnicastLocator, l);
        return p.Finish();
    }

    /// <summary>Decodes a remote endpoint announcement; guid, topic, and type are mandatory.</summary>
    public static bool TryDecode(byte[] payload, out EndpointData? data)
    {
        data = null;
        if (payload.Length < 4 || payload[0] != 0x00 || payload[1] != 0x03) return false; // PL_CDR_LE only

        var d = new EndpointData();
        var p = new ParameterListReader(payload, 0, payload.Length);
        while (p.Next(out ushort pid, out var v))
        {
            var r = new CdrReader(v.Array!, v.Offset, v.Count, hasEncapsulation: false);
            switch (pid)
            {
                case Pid.EndpointGuid when v.Count >= 16:
                    d.Guid = RtpsGuid.ReadFrom(v.AsSpan());
                    break;
                case Pid.TopicName when v.Count >= 4:
                    d.TopicName = r.ReadString();
                    break;
                case Pid.TypeName when v.Count >= 4:
                    d.TypeName = r.ReadString();
                    break;
                case Pid.Reliability when v.Count >= 4:
                    d.Reliable = r.ReadUInt32() >= 2;
                    break;
                case Pid.Durability when v.Count >= 4:
                    d.TransientLocal = r.ReadUInt32() >= 1;
                    break;
                case Pid.UnicastLocator when v.Count >= 24:
                    d.UnicastLocators.Add(Locator.ReadFrom(r));
                    break;
            }
        }

        if (d.Guid.Prefix == default || d.TopicName.Length == 0 || d.TypeName.Length == 0) return false;
        data = d;
        return true;
    }
}

/// <summary>The ROS 2 → DDS naming rules.</summary>
public static class Ros2Names
{
    /// <summary>"/chatter" → "rt/chatter".</summary>
    public static string Topic(string ros2Topic) => "rt" + Absolute(ros2Topic);

    /// <summary>"std_msgs/msg/String" → "std_msgs::msg::dds_::String_".</summary>
    public static string Type(string ros2Type)
    {
        string[] parts = ros2Type.Split('/');
        return parts.Length == 3 ? $"{parts[0]}::{parts[1]}::dds_::{parts[2]}_" : ros2Type;
    }

    /// <summary>"/add_two_ints" → "rq/add_two_intsRequest".</summary>
    public static string ServiceRequestTopic(string service) => "rq" + Absolute(service) + "Request";

    /// <summary>"/add_two_ints" → "rr/add_two_intsReply".</summary>
    public static string ServiceReplyTopic(string service) => "rr" + Absolute(service) + "Reply";

    /// <summary>"example_interfaces/srv/AddTwoInts" → "example_interfaces::srv::dds_::AddTwoInts_Request_".</summary>
    public static string ServiceRequestType(string srvType) => ServiceType(srvType, "Request");

    /// <summary>"example_interfaces/srv/AddTwoInts" → "example_interfaces::srv::dds_::AddTwoInts_Response_".</summary>
    public static string ServiceReplyType(string srvType) => ServiceType(srvType, "Response");

    private static string ServiceType(string srvType, string suffix)
    {
        string[] parts = srvType.Split('/');
        return parts.Length == 3 ? $"{parts[0]}::{parts[1]}::dds_::{parts[2]}_{suffix}_" : srvType;
    }

    private static string Absolute(string name) => name.StartsWith('/') ? name : "/" + name;
}
