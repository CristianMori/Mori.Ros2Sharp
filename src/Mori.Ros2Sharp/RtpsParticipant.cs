using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;

namespace Mori.Ros2Sharp;

/// <summary>
/// An RTPS participant: binds the well-known ports for a domain, runs SPDP participant
/// discovery and SEDP endpoint discovery, and hosts user readers/writers created with
/// <see cref="CreateReader"/> / <see cref="CreateWriter"/>.
/// </summary>
public sealed class RtpsParticipant : IDisposable
{
    private readonly UdpClient _meta;
    private readonly UdpClient _user;
    private readonly UdpClient _multicast;
    private readonly Dictionary<GuidPrefix, ParticipantData> _remote = new();
    private readonly Dictionary<RtpsGuid, EndpointData> _remotePublications = new();
    private readonly Dictionary<RtpsGuid, EndpointData> _remoteSubscriptions = new();
    private readonly List<RtpsWriterEndpoint> _writers = new();
    private readonly List<RtpsReaderEndpoint> _readers = new();
    private readonly List<IPAddress> _peers = new();
    private readonly CancellationTokenSource _cts = new();
    private readonly RtpsWriterEndpoint _sedpPublicationsWriter;
    private readonly RtpsWriterEndpoint _sedpSubscriptionsWriter;
    private readonly RtpsReaderEndpoint _sedpPublicationsReader;
    private readonly RtpsReaderEndpoint _sedpSubscriptionsReader;
    private long _spdpSequence;
    private int _entityKey;

    public int DomainId { get; }

    /// <summary>The participant slot whose well-known unicast ports we bound.</summary>
    public int ParticipantId { get; }

    public RtpsGuid Guid { get; }
    public string Name { get; }

    /// <summary>Opaque USER_DATA carried in our SPDP announcement (ROS 2 puts "enclave=…;" here).</summary>
    public byte[]? UserData { get; set; }

    /// <summary>
    /// The largest datagram we send. Fast DDS drops anything over 65,500 bytes, so this stays
    /// below that with room for the per-datagram submessage overhead.
    /// </summary>
    public const int MaxDatagramSize = 65_000;

    // RTPS header + INFO_DST + INFO_TS + the DATA_FRAG prologue and an inline-QoS related
    // identity, rounded up: what a datagram spends before the first payload byte.
    internal const int DatagramOverhead = 128;

    private int _fragmentSize = 64_000;

    /// <summary>
    /// The fragment size used when a sample does not fit one datagram. The default matches
    /// Fast DDS and is accepted by Cyclone DDS as well; a value around 1,400 keeps every
    /// datagram under the Ethernet MTU for links where IP fragmentation is a problem.
    /// Clamped so one fragment always fits a datagram.
    /// </summary>
    public int FragmentSize
    {
        get => _fragmentSize;
        set => _fragmentSize = Math.Clamp(value, 1024, MaxDatagramSize - DatagramOverhead);
    }

    /// <summary>The largest payload that still goes out as a single DATA submessage.</summary>
    internal int MaxUnfragmentedPayload => MaxDatagramSize - DatagramOverhead;

    /// <summary>
    /// The local addresses advertised in our locators, or null to choose automatically: the
    /// address that routes to each configured peer first, then the multicast route, then the
    /// remaining non-link-local addresses, capped at <see cref="MaxAdvertisedAddresses"/>.
    /// Set this on hosts with many virtual adapters when the automatic choice is wrong.
    /// </summary>
    public IReadOnlyList<IPAddress>? AdvertisedAddresses { get; set; }

    /// <summary>
    /// How many unicast addresses we advertise at most. Fast DDS keeps only the first four
    /// unicast locators of a remote participant by default, so a longer list can push the
    /// one usable address out of the set the peer actually sends to.
    /// </summary>
    public const int MaxAdvertisedAddresses = 4;

    // Test hooks: return true to drop the datagram. The probe uses them to simulate loss and
    // exercise the retransmission paths without a lossy network.
    internal Func<byte[], bool>? DropOutgoing;
    internal Func<byte[], bool>? DropIncoming;

    /// <summary>Raised the first time a remote participant is heard.</summary>
    public event Action<ParticipantData, IPEndPoint>? ParticipantDiscovered;

    /// <summary>Raised when a participant's lease expires without a fresh announcement.</summary>
    public event Action<ParticipantData>? ParticipantLost;

    /// <summary>Raised for every remote publication learned through SEDP.</summary>
    public event Action<EndpointData>? PublicationDiscovered;

    /// <summary>Raised for every remote subscription learned through SEDP.</summary>
    public event Action<EndpointData>? SubscriptionDiscovered;

    /// <summary>Raised when a remote publication is withdrawn, or its participant is lost.</summary>
    public event Action<EndpointData>? PublicationLost;

    /// <summary>Raised when a remote subscription is withdrawn, or its participant is lost.</summary>
    public event Action<EndpointData>? SubscriptionLost;

    /// <summary>
    /// Raised when handling a received datagram threw. The receive loop carries on regardless;
    /// this exists so such failures are visible instead of silently dropping traffic.
    /// </summary>
    public event Action<Exception>? ReceiveError;

    public RtpsParticipant(int domainId = 0, string? name = null)
    {
        DomainId = domainId;
        Name = name ?? "mori_ros2sharp";
        Guid = new RtpsGuid(GuidPrefix.NewUnique(), EntityId.Participant);

        // Claim the first participant slot with both well-known unicast ports free: metatraffic
        // (where unicast-only discovery peers answer) and user traffic.
        UdpClient? meta = null, user = null;
        int pid = 0;
        for (; pid < 120; pid++)
        {
            try { meta = NewUdp(RtpsPorts.MetatrafficUnicast(domainId, pid)); }
            catch (SocketException) { continue; }
            try { user = NewUdp(RtpsPorts.DefaultUnicast(domainId, pid)); break; }
            catch (SocketException) { meta.Dispose(); meta = null; }
        }
        _meta = meta ?? throw new InvalidOperationException("no free RTPS participant slot on this host");
        _user = user!;
        ParticipantId = pid;

        _multicast = NewUdp(RtpsPorts.MetatrafficMulticast(domainId), reuse: true);
        foreach (var ip in LocalAddresses())
        {
            try { _multicast.JoinMulticastGroup(RtpsPorts.DiscoveryMulticastGroup, ip); }
            catch (SocketException) { }
        }

        _sedpPublicationsWriter = new RtpsWriterEndpoint(this,
            new RtpsGuid(Guid.Prefix, EntityId.SedpPublicationsWriter), "", "",
            reliable: true, transientLocal: true, historyDepth: 64);
        _sedpSubscriptionsWriter = new RtpsWriterEndpoint(this,
            new RtpsGuid(Guid.Prefix, EntityId.SedpSubscriptionsWriter), "", "",
            reliable: true, transientLocal: true, historyDepth: 64);
        _sedpPublicationsReader = new RtpsReaderEndpoint(this,
            new RtpsGuid(Guid.Prefix, EntityId.SedpPublicationsReader), "", "", reliable: true);
        _sedpSubscriptionsReader = new RtpsReaderEndpoint(this,
            new RtpsGuid(Guid.Prefix, EntityId.SedpSubscriptionsReader), "", "", reliable: true);
        _sedpPublicationsReader.DataReceived += (_, payload, _) => OnRemotePublication(payload);
        _sedpSubscriptionsReader.DataReceived += (_, payload, _) => OnRemoteSubscription(payload);
        _sedpPublicationsReader.InstanceDisposed += (_, key) => RemoveRemotePublication(RtpsGuid.ReadFrom(key));
        _sedpSubscriptionsReader.InstanceDisposed += (_, key) => RemoveRemoteSubscription(RtpsGuid.ReadFrom(key));
    }

    /// <summary>Starts the discovery, receive, and reliability loops.</summary>
    public void Start()
    {
        _ = ReceiveLoop(_meta);
        _ = ReceiveLoop(_user);
        _ = ReceiveLoop(_multicast);
        _ = AnnounceLoop();
        _ = HeartbeatLoop();
    }

    /// <summary>Adds a host to announce to directly — discovery where multicast cannot reach.</summary>
    public void AddPeer(IPAddress address)
    {
        lock (_peers) _peers.Add(address);
    }

    /// <summary>A snapshot of the remote participants currently within their lease.</summary>
    public IReadOnlyList<ParticipantData> Participants
    {
        get { lock (_remote) return _remote.Values.ToList(); }
    }

    /// <summary>
    /// Creates a writer and announces it through SEDP. Names are DDS-level (see <see cref="Ros2Names"/>).
    /// A transient-local writer replays its history (the last <paramref name="historyDepth"/>
    /// samples) to readers that match later — the DDS form of a latched topic.
    /// </summary>
    public RtpsWriterEndpoint CreateWriter(string topicName, string typeName,
        bool reliable = true, bool transientLocal = false, int historyDepth = 32)
    {
        var guid = new RtpsGuid(Guid.Prefix, new EntityId((uint)(Interlocked.Increment(ref _entityKey) << 8) | 0x03));
        var writer = new RtpsWriterEndpoint(this, guid, topicName, typeName, reliable, transientLocal, historyDepth);
        List<EndpointData> subs;
        lock (_writers) _writers.Add(writer);
        lock (_remotePublications)
            subs = _remoteSubscriptions.Values.Where(s => s.TopicName == topicName && s.TypeName == typeName).ToList();
        foreach (var s in subs)
            if (QosCompatible(reliable, transientLocal, s.Reliable, s.TransientLocal))
                writer.MatchReader(s.Guid, ResolveLocators(s), s.TransientLocal);
        _sedpPublicationsWriter.WriteInstance(LocalEndpointData(guid, topicName, typeName, reliable, transientLocal).Encode(), guid);
        return writer;
    }

    /// <summary>Withdraws a local writer: SEDP dispose to every participant, no more routing to it.</summary>
    internal void RemoveWriter(RtpsWriterEndpoint writer)
    {
        bool removed;
        lock (_writers) removed = _writers.Remove(writer);
        if (!removed) return;
        _sedpPublicationsWriter.DisposeInstance(writer.Guid);
    }

    /// <summary>Withdraws a local reader: SEDP dispose to every participant, no more routing to it.</summary>
    internal void RemoveReader(RtpsReaderEndpoint reader)
    {
        bool removed;
        lock (_readers) removed = _readers.Remove(reader);
        if (!removed) return;
        _sedpSubscriptionsWriter.DisposeInstance(reader.Guid);
    }

    /// <summary>
    /// Creates a reader and announces it through SEDP. Names are DDS-level (see <see cref="Ros2Names"/>).
    /// A transient-local reader only matches transient-local writers, and receives what they
    /// wrote before it existed.
    /// </summary>
    public RtpsReaderEndpoint CreateReader(string topicName, string typeName, bool reliable = true, bool transientLocal = false)
    {
        var guid = new RtpsGuid(Guid.Prefix, new EntityId((uint)(Interlocked.Increment(ref _entityKey) << 8) | 0x04));
        var reader = new RtpsReaderEndpoint(this, guid, topicName, typeName, reliable, transientLocal);
        List<EndpointData> pubs;
        lock (_readers) _readers.Add(reader);
        lock (_remotePublications)
            pubs = _remotePublications.Values.Where(p => p.TopicName == topicName && p.TypeName == typeName).ToList();
        foreach (var p in pubs)
            if (QosCompatible(p.Reliable, p.TransientLocal, reliable, transientLocal))
                reader.MatchWriter(p.Guid, ResolveLocators(p));
        _sedpSubscriptionsWriter.WriteInstance(LocalEndpointData(guid, topicName, typeName, reliable, transientLocal).Encode(), guid);
        return reader;
    }

    // The DDS request/offer rule for the two QoS we implement: a reader may not ask for more
    // than the writer offers. Reliable readers need reliable writers; transient-local readers
    // need transient-local writers. The other direction is always fine.
    private static bool QosCompatible(bool writerReliable, bool writerTransientLocal, bool readerReliable, bool readerTransientLocal) =>
        (writerReliable || !readerReliable) && (writerTransientLocal || !readerTransientLocal);

    public void Dispose()
    {
        SendSpdpDispose();
        _cts.Cancel();
        _meta.Dispose();
        _user.Dispose();
        _multicast.Dispose();
        _cts.Dispose();
    }

    /// <summary>Tells the domain we are leaving so remotes drop us now instead of at lease expiry.</summary>
    private void SendSpdpDispose()
    {
        var w = new RtpsMessageWriter(Guid.Prefix);
        w.AddInfoTimestamp(RtpsTime.FromDateTime(DateTimeOffset.UtcNow));
        w.AddDisposeData(EntityId.SpdpReader, EntityId.SpdpWriter, Interlocked.Increment(ref _spdpSequence), Guid);
        byte[] datagram = w.ToArray();

        SendMulticast(datagram);
        var targets = new List<IPEndPoint>();
        lock (_peers)
            foreach (var peer in _peers)
                for (int pid = 0; pid < 4; pid++)
                    targets.Add(new IPEndPoint(peer, RtpsPorts.MetatrafficUnicast(DomainId, pid)));
        lock (_remote)
            foreach (var pd in _remote.Values)
                targets.AddRange(ToEndPoints(pd.MetatrafficUnicastLocators));

        foreach (var ep in targets)
        {
            try { _meta.Client.SendTo(datagram, ep); }
            catch (SocketException) { }
        }
    }

    /// <summary>Sends submessages scoped to one destination participant, to the given locators.</summary>
    internal void SendDirected(GuidPrefix destination, IReadOnlyList<IPEndPoint> targets, Action<RtpsMessageWriter> build)
    {
        var w = new RtpsMessageWriter(Guid.Prefix);
        w.AddInfoDestination(destination);
        build(w);
        byte[] datagram = w.ToArray();
        if (DropOutgoing?.Invoke(datagram) == true) return;
        foreach (var ep in targets)
        {
            try { _meta.Client.SendTo(datagram, ep); }
            catch (SocketException) { }
            catch (ObjectDisposedException) { return; } // disposed while a resend was under way
        }
    }

    private static UdpClient NewUdp(int port, bool reuse = false)
    {
        var udp = new UdpClient();
        if (reuse) udp.Client.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReuseAddress, true);
        // A fragmented sample arrives as a burst of near-64 KB datagrams; the OS default
        // buffers (64 KB on Windows) would drop most of it before we ever read it.
        try { udp.Client.ReceiveBufferSize = 4 << 20; } catch (SocketException) { }
        try { udp.Client.SendBufferSize = 4 << 20; } catch (SocketException) { }
        if (OperatingSystem.IsWindows())
        {
            // SIO_UDP_CONNRESET off: an ICMP port-unreachable from one peer must not fault
            // pending receives for everyone else.
            try { udp.Client.IOControl(-1744830452, new byte[] { 0 }, null); }
            catch (SocketException) { }
        }
        udp.Client.Bind(new IPEndPoint(IPAddress.Any, port));
        return udp;
    }

    // The addresses that go into our advertised locators, most likely to be reachable first.
    // A host with VPN, container, and hypervisor adapters can have a dozen addresses, and a
    // peer that only honors the first few must see the right one among them.
    private IReadOnlyList<IPAddress> AdvertisedLocalAddresses()
    {
        if (AdvertisedAddresses is { Count: > 0 } chosen) return chosen;
        var ordered = new List<IPAddress>();
        void Add(IPAddress? ip)
        {
            if (ip != null && !ip.Equals(IPAddress.Any) && !ordered.Contains(ip)) ordered.Add(ip);
        }
        lock (_peers)
            foreach (var peer in _peers)
                Add(RouteSource(peer));
        Add(RouteSource(RtpsPorts.DiscoveryMulticastGroup));
        foreach (var ip in LocalAddresses())
            if (!IsLinkLocal(ip))
                Add(ip);
        return ordered.Count > MaxAdvertisedAddresses ? ordered.GetRange(0, MaxAdvertisedAddresses) : ordered;
    }

    // The local address the routing table would use to reach the target (no packet is sent).
    private static IPAddress? RouteSource(IPAddress target)
    {
        try
        {
            using var s = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
            s.Connect(target, 7400);
            return (s.LocalEndPoint as IPEndPoint)?.Address;
        }
        catch (SocketException) { return null; }
    }

    private static bool IsLinkLocal(IPAddress ip)
    {
        byte[] b = ip.GetAddressBytes();
        return b.Length == 4 && b[0] == 169 && b[1] == 254;
    }

    // Every up, non-loopback IPv4 address: the multicast group is joined on each of them.
    private static IEnumerable<IPAddress> LocalAddresses()
    {
        foreach (var nic in NetworkInterface.GetAllNetworkInterfaces())
        {
            if (nic.OperationalStatus != OperationalStatus.Up ||
                nic.NetworkInterfaceType == NetworkInterfaceType.Loopback)
                continue;
            foreach (var ua in nic.GetIPProperties().UnicastAddresses)
                if (ua.Address.AddressFamily == AddressFamily.InterNetwork)
                    yield return ua.Address;
        }
    }

    // Only UDPv4 locators are usable by our transport; shared-memory and TCP kinds are dropped
    // here and the remote falls back to UDP.
    private static IReadOnlyList<IPEndPoint> ToEndPoints(IEnumerable<Locator> locators) =>
        locators.Where(l => l.Kind == Locator.KindUdpV4 && !l.Address.Equals(IPAddress.Any))
                .Select(l => new IPEndPoint(l.Address, (int)l.Port))
                .ToList();

    // Builtin endpoints first, then a snapshot of the user endpoints (snapshotting keeps the
    // receive path safe against concurrent CreateWriter/CreateReader calls).
    private IEnumerable<RtpsWriterEndpoint> AllWriters()
    {
        yield return _sedpPublicationsWriter;
        yield return _sedpSubscriptionsWriter;
        List<RtpsWriterEndpoint> users;
        lock (_writers) users = _writers.ToList();
        foreach (var w in users) yield return w;
    }

    private IEnumerable<RtpsReaderEndpoint> AllReaders()
    {
        yield return _sedpPublicationsReader;
        yield return _sedpSubscriptionsReader;
        List<RtpsReaderEndpoint> users;
        lock (_readers) users = _readers.ToList();
        foreach (var r in users) yield return r;
    }

    /// <summary>Our own SPDP announcement, rebuilt each period so address changes are picked up.</summary>
    private ParticipantData BuildLocalData()
    {
        var data = new ParticipantData
        {
            Guid = Guid,
            BuiltinEndpoints =
                BuiltinEndpoints.ParticipantAnnouncer | BuiltinEndpoints.ParticipantDetector |
                BuiltinEndpoints.PublicationsAnnouncer | BuiltinEndpoints.PublicationsDetector |
                BuiltinEndpoints.SubscriptionsAnnouncer | BuiltinEndpoints.SubscriptionsDetector,
            DomainId = DomainId,
            EntityName = Name,
            UserData = UserData,
        };
        foreach (var ip in AdvertisedLocalAddresses())
        {
            data.MetatrafficUnicastLocators.Add(Locator.UdpV4(ip, RtpsPorts.MetatrafficUnicast(DomainId, ParticipantId)));
            data.DefaultUnicastLocators.Add(Locator.UdpV4(ip, RtpsPorts.DefaultUnicast(DomainId, ParticipantId)));
        }
        data.MetatrafficMulticastLocators.Add(
            Locator.UdpV4(RtpsPorts.DiscoveryMulticastGroup, RtpsPorts.MetatrafficMulticast(DomainId)));
        return data;
    }

    /// <summary>The SEDP announcement for one of our endpoints (user traffic goes to the default ports).</summary>
    private EndpointData LocalEndpointData(RtpsGuid guid, string topicName, string typeName,
        bool reliable, bool transientLocal)
    {
        var e = new EndpointData
        {
            Guid = guid,
            TopicName = topicName,
            TypeName = typeName,
            Reliable = reliable,
            TransientLocal = transientLocal,
        };
        foreach (var ip in AdvertisedLocalAddresses())
            e.UnicastLocators.Add(Locator.UdpV4(ip, RtpsPorts.DefaultUnicast(DomainId, ParticipantId)));
        return e;
    }

    // SPDP cadence: announce every 3 s (well inside our advertised 20 s lease) and reap
    // remotes whose own leases have lapsed.
    private async Task AnnounceLoop()
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(3));
        try
        {
            do
            {
                await AnnounceAsync();
                PruneExpired();
            }
            while (await timer.WaitForNextTickAsync(_cts.Token));
        }
        catch (OperationCanceledException) { }
    }

    // Reliability cadence: every second, each reliable writer nudges readers that have not
    // acknowledged its full history yet. Retransmission itself is ACKNACK-driven.
    private async Task HeartbeatLoop()
    {
        using var timer = new PeriodicTimer(TimeSpan.FromSeconds(1));
        try
        {
            while (await timer.WaitForNextTickAsync(_cts.Token))
                foreach (var w in AllWriters())
                    w.SendHeartbeats();
        }
        catch (OperationCanceledException) { }
    }

    /// <summary>One SPDP announcement, to the multicast group and to every configured peer.</summary>
    private async Task AnnounceAsync()
    {
        var w = new RtpsMessageWriter(Guid.Prefix);
        w.AddInfoTimestamp(RtpsTime.FromDateTime(DateTimeOffset.UtcNow));
        w.AddData(EntityId.SpdpReader, EntityId.SpdpWriter, ++_spdpSequence, BuildLocalData().Encode());
        byte[] datagram = w.ToArray();

        SendMulticast(datagram);
        var targets = new List<IPEndPoint>();
        lock (_peers)
        {
            // The first few participant slots per peer host, as DDS initial-peer lists do.
            foreach (var peer in _peers)
                for (int pid = 0; pid < 4; pid++)
                    targets.Add(new IPEndPoint(peer, RtpsPorts.MetatrafficUnicast(DomainId, pid)));
        }

        foreach (var ep in targets)
        {
            try { await _meta.SendAsync(datagram, ep, _cts.Token); }
            catch (SocketException) { }
        }
    }

    /// <summary>
    /// Sends one datagram to the discovery multicast group once per local interface. A
    /// single send would leave through the default route only, which on a host with VPN or
    /// virtual adapters is rarely the interface the other participants are on.
    /// </summary>
    private void SendMulticast(byte[] datagram)
    {
        var group = new IPEndPoint(RtpsPorts.DiscoveryMulticastGroup, RtpsPorts.MetatrafficMulticast(DomainId));
        foreach (var ip in LocalAddresses())
        {
            try
            {
                _meta.Client.SetSocketOption(SocketOptionLevel.IP, SocketOptionName.MulticastInterface, ip.GetAddressBytes());
                _meta.Client.SendTo(datagram, group);
            }
            catch (SocketException) { }
            catch (ObjectDisposedException) { return; }
        }
    }

    // One loop per socket (metatraffic unicast, user unicast, discovery multicast). Socket
    // errors on individual datagrams must never end the loop; only cancellation/dispose does.
    private async Task ReceiveLoop(UdpClient udp)
    {
        while (!_cts.IsCancellationRequested)
        {
            UdpReceiveResult r;
            try { r = await udp.ReceiveAsync(_cts.Token); }
            catch (OperationCanceledException) { break; }
            catch (ObjectDisposedException) { break; }
            catch (SocketException) { continue; }
            if (DropIncoming?.Invoke(r.Buffer) == true) continue;
            try { Handle(r.Buffer, r.RemoteEndPoint); }
            catch (Exception ex) { ReceiveError?.Invoke(ex); } // a bad datagram must not kill the loop
        }
    }

    /// <summary>
    /// Routes one parsed datagram: SPDP data is handled here, everything else fans out to the
    /// endpoints, which decide relevance by matched writer guid / entity id.
    /// </summary>
    private void Handle(byte[] datagram, IPEndPoint from)
    {
        // Drop our own multicast loopback and anything scoped to a different participant.
        if (!RtpsMessage.TryParse(datagram, out var msg) || msg!.Source == Guid.Prefix) return;
        if (msg.Destination is { } dst && dst != default(GuidPrefix) && dst != Guid.Prefix) return;

        // Every reader that matches the writer gets the sample: a DATA addressed to one reader
        // (readerId set) is taken by that reader alone, a broadcast one (readerId unknown) by
        // all of them — two subscriptions to one topic in one participant are both served.
        foreach (var d in msg.Data)
        {
            if (d.WriterId == EntityId.SpdpWriter) { HandleSpdp(d, from); continue; }
            foreach (var r in AllReaders())
                r.OnData(msg.Source, d);
        }
        foreach (var f in msg.DataFrags)
        {
            if (f.WriterId == EntityId.SpdpWriter) continue; // announcements are never fragmented
            foreach (var r in AllReaders())
                r.OnDataFrag(msg.Source, f);
        }
        foreach (var hb in msg.Heartbeats)
            foreach (var r in AllReaders())
                r.OnHeartbeat(msg.Source, hb);
        foreach (var hf in msg.HeartbeatFrags)
            foreach (var r in AllReaders())
                r.OnHeartbeatFrag(msg.Source, hf);
        foreach (var gap in msg.Gaps)
            foreach (var r in AllReaders())
                r.OnGap(msg.Source, gap);
        foreach (var an in msg.AckNacks)
            foreach (var w in AllWriters())
                w.OnAckNack(msg.Source, an);
        foreach (var nf in msg.NackFrags)
            foreach (var w in AllWriters())
                w.OnNackFrag(msg.Source, nf);
    }

    /// <summary>A remote SPDP sample: a fresh/refreshed announcement, or a departure dispose.</summary>
    private void HandleSpdp(RtpsData d, IPEndPoint from)
    {
        if (d.Payload.Length == 0)
        {
            // A dispose: the participant named by the inline-QoS key hash is leaving.
            if (!d.Disposed || d.KeyHash is not { Length: >= 12 }) return;
            var prefix = GuidPrefix.ReadFrom(d.KeyHash);
            ParticipantData? gone;
            lock (_remote)
                if (_remote.TryGetValue(prefix, out gone))
                    _remote.Remove(prefix);
            if (gone != null) ForgetParticipant(gone);
            return;
        }
        if (!ParticipantData.TryDecode(d.Payload, out var pd)) return;
        pd!.LastSeen = DateTimeOffset.UtcNow;
        bool isNew;
        lock (_remote)
        {
            isNew = !_remote.ContainsKey(pd.Guid.Prefix);
            _remote[pd.Guid.Prefix] = pd;
        }
        MatchBuiltins(pd);
        if (isNew) ParticipantDiscovered?.Invoke(pd, from);
    }

    /// <summary>
    /// Pairs our SEDP builtins with the remote's, per its advertised BuiltinEndpointSet bits.
    /// Runs on every refresh so locator changes propagate; matching is idempotent.
    /// </summary>
    private void MatchBuiltins(ParticipantData pd)
    {
        var meta = ToEndPoints(pd.MetatrafficUnicastLocators);
        if (meta.Count == 0) return;
        var be = pd.BuiltinEndpoints;
        var prefix = pd.Guid.Prefix;
        // The SEDP builtin readers are transient-local by specification.
        if (be.HasFlag(BuiltinEndpoints.PublicationsDetector))
            _sedpPublicationsWriter.MatchReader(new RtpsGuid(prefix, EntityId.SedpPublicationsReader), meta, readerTransientLocal: true);
        if (be.HasFlag(BuiltinEndpoints.SubscriptionsDetector))
            _sedpSubscriptionsWriter.MatchReader(new RtpsGuid(prefix, EntityId.SedpSubscriptionsReader), meta, readerTransientLocal: true);
        if (be.HasFlag(BuiltinEndpoints.PublicationsAnnouncer))
            _sedpPublicationsReader.MatchWriter(new RtpsGuid(prefix, EntityId.SedpPublicationsWriter), meta);
        if (be.HasFlag(BuiltinEndpoints.SubscriptionsAnnouncer))
            _sedpSubscriptionsReader.MatchWriter(new RtpsGuid(prefix, EntityId.SedpSubscriptionsWriter), meta);
    }

    /// <summary>A remote writer appeared: remember it and match compatible local readers.</summary>
    private void OnRemotePublication(byte[] payload)
    {
        if (!EndpointData.TryDecode(payload, out var e)) return;
        List<RtpsReaderEndpoint> matches;
        lock (_remotePublications) _remotePublications[e!.Guid] = e;
        lock (_readers)
            matches = _readers.Where(r => r.TopicName == e.TopicName && r.TypeName == e.TypeName).ToList();
        var locators = ResolveLocators(e);
        foreach (var r in matches)
            if (QosCompatible(e.Reliable, e.TransientLocal, r.Reliable, r.TransientLocal))
                r.MatchWriter(e.Guid, locators);
        PublicationDiscovered?.Invoke(e);
    }

    /// <summary>A remote reader appeared: remember it and match compatible local writers.</summary>
    private void OnRemoteSubscription(byte[] payload)
    {
        if (!EndpointData.TryDecode(payload, out var e)) return;
        List<RtpsWriterEndpoint> matches;
        lock (_remotePublications) _remoteSubscriptions[e!.Guid] = e;
        lock (_writers)
            matches = _writers.Where(w => w.TopicName == e.TopicName && w.TypeName == e.TypeName).ToList();
        var locators = ResolveLocators(e);
        foreach (var w in matches)
            if (QosCompatible(w.Reliable, w.TransientLocal, e.Reliable, e.TransientLocal))
                w.MatchReader(e.Guid, locators, e.TransientLocal);
        SubscriptionDiscovered?.Invoke(e);
    }

    /// <summary>The endpoint's own locators, falling back to its participant's SPDP locators.</summary>
    private IReadOnlyList<IPEndPoint> ResolveLocators(EndpointData e)
    {
        var own = ToEndPoints(e.UnicastLocators);
        if (own.Count > 0) return own;
        lock (_remote)
            if (_remote.TryGetValue(e.Guid.Prefix, out var pd))
                return ToEndPoints(pd.DefaultUnicastLocators.Count > 0
                    ? pd.DefaultUnicastLocators
                    : pd.MetatrafficUnicastLocators);
        return Array.Empty<IPEndPoint>();
    }

    /// <summary>Drops remotes silent for 1.5× their lease (participants that died without a dispose).</summary>
    private void PruneExpired()
    {
        List<ParticipantData>? lost = null;
        var now = DateTimeOffset.UtcNow;
        lock (_remote)
        {
            foreach (var pd in _remote.Values)
            {
                if (pd.LeaseDuration.Seconds >= int.MaxValue) continue; // infinite lease
                double lease = Math.Max(pd.LeaseDuration.ToSeconds(), 5);
                if ((now - pd.LastSeen).TotalSeconds > lease * 1.5)
                    (lost ??= new()).Add(pd);
            }
            if (lost != null)
                foreach (var pd in lost)
                    _remote.Remove(pd.Guid.Prefix);
        }
        if (lost != null)
            foreach (var pd in lost)
                ForgetParticipant(pd);
    }

    /// <summary>
    /// A participant is gone (dispose or lease expiry): every endpoint of its is withdrawn,
    /// every local endpoint unmatched from it, builtins included, then ParticipantLost.
    /// </summary>
    private void ForgetParticipant(ParticipantData pd)
    {
        var prefix = pd.Guid.Prefix;
        List<EndpointData> pubs, subs;
        lock (_remotePublications)
        {
            pubs = _remotePublications.Values.Where(e => e.Guid.Prefix == prefix).ToList();
            subs = _remoteSubscriptions.Values.Where(e => e.Guid.Prefix == prefix).ToList();
        }
        foreach (var e in pubs) RemoveRemotePublication(e.Guid);
        foreach (var e in subs) RemoveRemoteSubscription(e.Guid);
        foreach (var w in AllWriters()) w.UnmatchParticipant(prefix);
        foreach (var r in AllReaders()) r.UnmatchParticipant(prefix);
        ParticipantLost?.Invoke(pd);
    }

    /// <summary>A remote writer is gone: unmatch it from every local reader.</summary>
    private void RemoveRemotePublication(RtpsGuid guid)
    {
        EndpointData? e;
        lock (_remotePublications)
            if (!_remotePublications.Remove(guid, out e)) return;
        foreach (var r in AllReaders()) r.UnmatchWriter(guid);
        PublicationLost?.Invoke(e!);
    }

    /// <summary>A remote reader is gone: unmatch it from every local writer.</summary>
    private void RemoveRemoteSubscription(RtpsGuid guid)
    {
        EndpointData? e;
        lock (_remotePublications)
            if (!_remoteSubscriptions.Remove(guid, out e)) return;
        foreach (var w in AllWriters()) w.UnmatchReader(guid);
        SubscriptionLost?.Invoke(e!);
    }
}
