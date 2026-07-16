using System.Net;
using System.Text;

namespace Mori.Ros2Sharp;

/// <summary>
/// A ROS 2 node: an RTPS participant plus the ROS graph conventions on top — name mangling,
/// the enclave user data, and ros_discovery_info participation so the command-line tools
/// (ros2 node list, ros2 node info) see this process as a node.
/// </summary>
public sealed class Ros2Node : IDisposable
{
    private readonly List<RtpsGuid> _readerGids = new();
    private readonly List<RtpsGuid> _writerGids = new();
    private readonly RtpsWriterEndpoint _discoveryInfo;
    private readonly object _lock = new();

    public string NodeName { get; }
    public string Namespace { get; }

    /// <summary>The underlying participant, for RTPS-level access (events, peers, raw endpoints).</summary>
    public RtpsParticipant Participant { get; }

    public Ros2Node(string nodeName, string ns = "/", int domainId = 0)
    {
        NodeName = nodeName;
        Namespace = ns;
        Participant = new RtpsParticipant(domainId, "/") // participant name = enclave, as rmw does
        {
            UserData = Encoding.UTF8.GetBytes("enclave=/;"),
        };
        _discoveryInfo = Participant.CreateWriter("ros_discovery_info",
            "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_",
            reliable: true, transientLocal: true, historyDepth: 1);
        PublishGraph();
    }

    public void Start() => Participant.Start();

    public void AddPeer(IPAddress address) => Participant.AddPeer(address);

    /// <summary>Creates a publisher from ROS names, e.g. ("/chatter", "std_msgs/msg/String").</summary>
    public RtpsWriterEndpoint CreatePublisher(string topic, string type, bool reliable = true)
    {
        var writer = Participant.CreateWriter(Ros2Names.Topic(topic), Ros2Names.Type(type), reliable);
        lock (_lock) _writerGids.Add(writer.Guid);
        PublishGraph();
        return writer;
    }

    /// <summary>Creates a subscription from ROS names, e.g. ("/chatter", "std_msgs/msg/String").</summary>
    public RtpsReaderEndpoint CreateSubscription(string topic, string type, bool reliable = true)
    {
        var reader = Participant.CreateReader(Ros2Names.Topic(topic), Ros2Names.Type(type), reliable);
        lock (_lock) _readerGids.Add(reader.Guid);
        PublishGraph();
        return reader;
    }

    /// <summary>Serves a service, e.g. ("/add_two_ints", "example_interfaces/srv/AddTwoInts", …).</summary>
    public Ros2Service CreateService(string serviceName, string serviceType, Func<byte[], byte[]> handler)
    {
        var requests = Participant.CreateReader(
            Ros2Names.ServiceRequestTopic(serviceName), Ros2Names.ServiceRequestType(serviceType));
        var replies = Participant.CreateWriter(
            Ros2Names.ServiceReplyTopic(serviceName), Ros2Names.ServiceReplyType(serviceType));
        lock (_lock)
        {
            _readerGids.Add(requests.Guid);
            _writerGids.Add(replies.Guid);
        }
        PublishGraph();
        return new Ros2Service(serviceName, requests, replies, handler);
    }

    /// <summary>Creates a client for a service, e.g. ("/add_two_ints", "example_interfaces/srv/AddTwoInts").</summary>
    public Ros2Client CreateClient(string serviceName, string serviceType)
    {
        var requests = Participant.CreateWriter(
            Ros2Names.ServiceRequestTopic(serviceName), Ros2Names.ServiceRequestType(serviceType));
        var replies = Participant.CreateReader(
            Ros2Names.ServiceReplyTopic(serviceName), Ros2Names.ServiceReplyType(serviceType));
        lock (_lock)
        {
            _writerGids.Add(requests.Guid);
            _readerGids.Add(replies.Guid);
        }
        PublishGraph();
        return new Ros2Client(serviceName, requests, replies);
    }

    /// <summary>
    /// Serializes rmw_dds_common/msg/ParticipantEntitiesInfo: the participant gid and, per node,
    /// namespace, name, and reader/writer gid lists. Gids are 24 bytes (the RTPS GUID zero-padded).
    /// </summary>
    public static byte[] EncodeParticipantEntitiesInfo(RtpsGuid participant, string ns,
        string nodeName, IReadOnlyList<RtpsGuid> readerGids, IReadOnlyList<RtpsGuid> writerGids)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        WriteGid(w, participant);
        w.WriteLength(1); // one node on this participant
        w.Write(ns);
        w.Write(nodeName);
        w.WriteLength(readerGids.Count);
        foreach (var g in readerGids) WriteGid(w, g);
        w.WriteLength(writerGids.Count);
        foreach (var g in writerGids) WriteGid(w, g);
        return w.ToArray();
    }

    // Re-announces this node's entity lists whenever they change. The writer is transient-local
    // with depth 1: late joiners always receive exactly the current state.
    private void PublishGraph()
    {
        byte[] payload;
        lock (_lock)
            payload = EncodeParticipantEntitiesInfo(Participant.Guid, Namespace, NodeName, _readerGids, _writerGids);
        _discoveryInfo.Write(payload);
    }

    // A gid is the 16-byte RTPS GUID zero-padded to the fixed 24-byte field.
    private static void WriteGid(CdrWriter w, RtpsGuid guid)
    {
        Span<byte> b = stackalloc byte[24];
        guid.WriteTo(b);
        w.WriteBytes(b);
    }

    public void Dispose() => Participant.Dispose();
}
