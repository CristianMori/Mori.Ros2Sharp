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

    /// <summary>
    /// Creates a publisher from ROS names, e.g. ("/chatter", "std_msgs/msg/String").
    /// <paramref name="transientLocal"/> makes it a latched topic: the last
    /// <paramref name="historyDepth"/> messages are delivered to subscriptions that appear
    /// later (what /tf_static and map publishers use).
    /// </summary>
    public RtpsWriterEndpoint CreatePublisher(string topic, string type, bool reliable = true,
        bool transientLocal = false, int historyDepth = 10)
    {
        var writer = Participant.CreateWriter(Ros2Names.Topic(topic), Ros2Names.Type(type), reliable, transientLocal, historyDepth);
        lock (_lock) _writerGids.Add(writer.Guid);
        PublishGraph();
        return writer;
    }

    /// <summary>
    /// Creates a subscription from ROS names, e.g. ("/chatter", "std_msgs/msg/String").
    /// <paramref name="transientLocal"/> requests latched history and restricts matching to
    /// transient-local publishers, as the DDS durability rule requires.
    /// </summary>
    public RtpsReaderEndpoint CreateSubscription(string topic, string type, bool reliable = true, bool transientLocal = false)
    {
        var reader = Participant.CreateReader(Ros2Names.Topic(topic), Ros2Names.Type(type), reliable, transientLocal);
        lock (_lock) _readerGids.Add(reader.Guid);
        PublishGraph();
        return reader;
    }

    /// <summary>Withdraws a publisher created by <see cref="CreatePublisher"/> and updates the graph.</summary>
    public void RemovePublisher(RtpsWriterEndpoint publisher)
    {
        publisher.Dispose();
        lock (_lock) _writerGids.Remove(publisher.Guid);
        PublishGraph();
    }

    /// <summary>Withdraws a subscription created by <see cref="CreateSubscription"/> and updates the graph.</summary>
    public void RemoveSubscription(RtpsReaderEndpoint subscription)
    {
        subscription.Dispose();
        lock (_lock) _readerGids.Remove(subscription.Guid);
        PublishGraph();
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

    /// <summary>
    /// Serves a service with generated request/response classes, e.g.
    /// <c>CreateService&lt;SetBool.Request, SetBool.Response&gt;("/set_bool", SetBool.RosType, req => …)</c>.
    /// </summary>
    public Ros2Service CreateService<TRequest, TResponse>(string serviceName, string serviceType, Func<TRequest, TResponse> handler)
        where TRequest : IRos2Message, new()
        where TResponse : IRos2Message =>
        CreateService(serviceName, serviceType,
            request => handler(Ros2MessageExtensions.FromPayload<TRequest>(request)).ToPayload());

    /// <summary>Creates a typed client for a service (see <see cref="Ros2Client{TRequest, TResponse}"/>).</summary>
    public Ros2Client<TRequest, TResponse> CreateClient<TRequest, TResponse>(string serviceName, string serviceType)
        where TRequest : IRos2Message
        where TResponse : IRos2Message, new() =>
        new(CreateClient(serviceName, serviceType));

    /// <summary>
    /// Creates an action client from generated types, e.g.
    /// <c>CreateActionClient&lt;Wait.Goal, Wait.Result, Wait.Feedback&gt;("/wait", Wait.RosType)</c>.
    /// </summary>
    public Ros2ActionClient<TGoal, TResult, TFeedback> CreateActionClient<TGoal, TResult, TFeedback>(string actionName, string actionType)
        where TGoal : IRos2Message
        where TResult : IRos2Message, new()
        where TFeedback : IRos2Message, new() =>
        new(this, actionName, actionType);

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
