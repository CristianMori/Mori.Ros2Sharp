using System.Net;

namespace Mori.Ros2Sharp;

/// <summary>
/// A local RTPS writer: keeps a bounded history, pushes DATA to matched readers, and answers
/// the reliability protocol (periodic HEARTBEAT, retransmit or GAP on ACKNACK).
/// </summary>
public sealed class RtpsWriterEndpoint
{
    private readonly RtpsParticipant _participant;
    private readonly List<Sample> _history = new();
    private readonly List<Remote> _readers = new();
    private readonly int _depth;
    private readonly object _lock = new();
    private long _nextSn = 1;
    private uint _heartbeatCount;

    public RtpsGuid Guid { get; }
    public string TopicName { get; }
    public string TypeName { get; }
    public bool Reliable { get; }
    public bool TransientLocal { get; }

    public int MatchedReaderCount { get { lock (_lock) return _readers.Count; } }

    private sealed record Sample(long Sn, byte[] Payload, RtpsTime Timestamp, RtpsGuid? RelatedGuid, long RelatedSn);

    private sealed class Remote
    {
        public RtpsGuid Guid;
        public IReadOnlyList<IPEndPoint> Locators = Array.Empty<IPEndPoint>();
        public long AckedBefore = 1; // every sequence number below this is acknowledged
    }

    internal RtpsWriterEndpoint(RtpsParticipant participant, RtpsGuid guid, string topicName,
        string typeName, bool reliable, bool transientLocal, int historyDepth)
    {
        _participant = participant;
        Guid = guid;
        TopicName = topicName;
        TypeName = typeName;
        Reliable = reliable;
        TransientLocal = transientLocal;
        _depth = historyDepth;
    }

    /// <summary>Writes one serialized payload (starting with its encapsulation header).</summary>
    public long Write(byte[] payload) => WriteCore(payload, null, 0, selfRelated: false);

    /// <summary>Writes with a related sample identity in inline QoS (service replies).</summary>
    public long Write(byte[] payload, RtpsGuid relatedGuid, long relatedSn) =>
        WriteCore(payload, relatedGuid, relatedSn, selfRelated: false);

    /// <summary>Writes carrying the sample's own identity as the related one (service requests).</summary>
    public long WriteSelfRelated(byte[] payload) => WriteCore(payload, null, 0, selfRelated: true);

    private long WriteCore(byte[] payload, RtpsGuid? relatedGuid, long relatedSn, bool selfRelated)
    {
        Sample s;
        List<Remote> readers;
        lock (_lock)
        {
            long sn = _nextSn++;
            if (selfRelated) { relatedGuid = Guid; relatedSn = sn; }
            s = new Sample(sn, payload, RtpsTime.FromDateTime(DateTimeOffset.UtcNow), relatedGuid, relatedSn);
            _history.Add(s);
            if (_history.Count > _depth) _history.RemoveAt(0);
            readers = _readers.ToList();
        }
        foreach (var r in readers) SendData(r, s);
        if (Reliable) SendHeartbeats();
        return s.Sn;
    }

    /// <summary>
    /// Registers (or re-locates) a remote reader. New transient-local matches get the whole
    /// history replayed — that is how SEDP data reaches late-joining participants.
    /// </summary>
    internal void MatchReader(RtpsGuid readerGuid, IReadOnlyList<IPEndPoint> locators)
    {
        Remote fresh;
        List<Sample>? replay = null;
        lock (_lock)
        {
            var existing = _readers.FirstOrDefault(r => r.Guid == readerGuid);
            if (existing != null) { existing.Locators = locators; return; }
            fresh = new Remote { Guid = readerGuid, Locators = locators };
            _readers.Add(fresh);
            if (TransientLocal) replay = _history.ToList();
        }
        if (replay != null)
            foreach (var s in replay)
                SendData(fresh, s);
        if (Reliable) SendHeartbeats();
    }

    /// <summary>
    /// The reader acked everything below Base and requested the Missing set: record the ack,
    /// retransmit what history still holds, and GAP whatever fell out of it.
    /// </summary>
    internal void OnAckNack(GuidPrefix source, RtpsAckNack an)
    {
        if (an.WriterId != Guid.Entity) return;
        var readerGuid = new RtpsGuid(source, an.ReaderId);
        Remote? reader;
        var resend = new List<Sample>();
        long firstAvailable;
        lock (_lock)
        {
            reader = _readers.FirstOrDefault(r => r.Guid == readerGuid);
            if (reader == null) return;
            reader.AckedBefore = Math.Max(reader.AckedBefore, an.BaseSn);
            firstAvailable = _history.Count > 0 ? _history[0].Sn : _nextSn;
            foreach (long sn in an.Missing)
            {
                var s = _history.FirstOrDefault(x => x.Sn == sn);
                if (s != null) resend.Add(s);
            }
        }
        foreach (var s in resend) SendData(reader, s);
        if (an.Missing.Any(sn => sn < firstAvailable))
            _participant.SendDirected(readerGuid.Prefix, reader.Locators,
                w => w.AddGap(readerGuid.Entity, Guid.Entity, an.BaseSn, firstAvailable));
    }

    /// <summary>One reliability tick: HEARTBEAT every matched reader that has not acked everything.</summary>
    internal void SendHeartbeats()
    {
        if (!Reliable) return;
        long first, last;
        uint count;
        List<Remote> pending;
        lock (_lock)
        {
            if (_history.Count == 0) return;
            first = _history[0].Sn;
            last = _history[^1].Sn;
            pending = _readers.Where(r => r.AckedBefore <= last).ToList();
            if (pending.Count == 0) return;
            count = ++_heartbeatCount;
        }
        foreach (var r in pending)
            _participant.SendDirected(r.Guid.Prefix, r.Locators,
                w => w.AddHeartbeat(r.Guid.Entity, Guid.Entity, first, last, count, final: false));
    }

    private void SendData(Remote reader, Sample s) =>
        _participant.SendDirected(reader.Guid.Prefix, reader.Locators, w =>
        {
            w.AddInfoTimestamp(s.Timestamp);
            if (s.RelatedGuid is { } related)
                w.AddData(reader.Guid.Entity, Guid.Entity, s.Sn, s.Payload, related, s.RelatedSn);
            else
                w.AddData(reader.Guid.Entity, Guid.Entity, s.Sn, s.Payload);
        });
}

/// <summary>
/// A local RTPS reader: accepts DATA from matched writers (with duplicate suppression), answers
/// HEARTBEATs with ACKNACKs, and honors GAPs.
/// </summary>
public sealed class RtpsReaderEndpoint
{
    private readonly RtpsParticipant _participant;
    private readonly Dictionary<RtpsGuid, Remote> _writers = new();
    private readonly object _lock = new();

    public RtpsGuid Guid { get; }
    public string TopicName { get; }
    public string TypeName { get; }
    public bool Reliable { get; }

    /// <summary>Raised once per new sample: writer guid, serialized payload, source timestamp.</summary>
    public event Action<RtpsGuid, byte[], RtpsTime?>? DataReceived;

    /// <summary>Raised once per new sample with full submessage detail (services use the identities).</summary>
    public event Action<RtpsGuid, RtpsData>? SampleReceived;

    public int MatchedWriterCount { get { lock (_lock) return _writers.Count; } }

    private sealed class Remote
    {
        public IReadOnlyList<IPEndPoint> Locators = Array.Empty<IPEndPoint>();
        public long ContiguousBefore = 1;               // every sn below this was seen or gapped
        public readonly HashSet<long> Received = new(); // sns at/above ContiguousBefore, out of order
        public uint AckNackCount;
    }

    internal RtpsReaderEndpoint(RtpsParticipant participant, RtpsGuid guid, string topicName,
        string typeName, bool reliable)
    {
        _participant = participant;
        Guid = guid;
        TopicName = topicName;
        TypeName = typeName;
        Reliable = reliable;
    }

    /// <summary>Registers (or re-locates) a remote writer; unmatched writers' DATA is ignored.</summary>
    internal void MatchWriter(RtpsGuid writerGuid, IReadOnlyList<IPEndPoint> locators)
    {
        lock (_lock)
        {
            if (_writers.TryGetValue(writerGuid, out var r)) r.Locators = locators;
            else _writers[writerGuid] = new Remote { Locators = locators };
        }
    }

    /// <summary>Returns true when the DATA belonged to one of this reader's matched writers.</summary>
    internal bool OnData(GuidPrefix source, RtpsData d)
    {
        var writerGuid = new RtpsGuid(source, d.WriterId);
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return false;
            if (d.Payload.Length == 0) return true; // dispose/unregister — endpoint removal, later
            if (d.SequenceNumber < r.ContiguousBefore || !r.Received.Add(d.SequenceNumber)) return true;
            Advance(r);
        }
        DataReceived?.Invoke(writerGuid, d.Payload, d.Timestamp);
        SampleReceived?.Invoke(writerGuid, d);
        return true;
    }

    /// <summary>
    /// Answers a writer's HEARTBEAT with an ACKNACK: ack everything contiguous, request the
    /// holes up to LastSn. Silence is only allowed when nothing is missing and the heartbeat
    /// was final.
    /// </summary>
    internal void OnHeartbeat(GuidPrefix source, RtpsHeartbeat hb)
    {
        if (hb.ReaderId != EntityId.Unknown && hb.ReaderId != Guid.Entity) return;
        var writerGuid = new RtpsGuid(source, hb.WriterId);
        long baseSn;
        var missing = new List<long>();
        uint count;
        IReadOnlyList<IPEndPoint> locators;
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return;
            if (r.ContiguousBefore < hb.FirstSn) // samples below FirstSn are no longer available
            {
                r.ContiguousBefore = hb.FirstSn;
                r.Received.RemoveWhere(sn => sn < hb.FirstSn);
                Advance(r);
            }
            baseSn = r.ContiguousBefore;
            for (long sn = baseSn; sn <= hb.LastSn && missing.Count < 256; sn++)
                if (!r.Received.Contains(sn))
                    missing.Add(sn);
            if (missing.Count == 0 && hb.Final) return;
            count = ++r.AckNackCount;
            locators = r.Locators;
        }
        _participant.SendDirected(source, locators,
            w => w.AddAckNack(Guid.Entity, hb.WriterId, baseSn, missing, count, final: missing.Count == 0));
    }

    /// <summary>Marks GAP-listed sequence numbers as seen so we stop requesting them.</summary>
    internal void OnGap(GuidPrefix source, RtpsGap gap)
    {
        if (gap.ReaderId != EntityId.Unknown && gap.ReaderId != Guid.Entity) return;
        var writerGuid = new RtpsGuid(source, gap.WriterId);
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return;
            for (long sn = gap.GapStart; sn < gap.GapListBase; sn++)
                if (sn >= r.ContiguousBefore)
                    r.Received.Add(sn);
            foreach (long sn in gap.Bits)
                if (sn >= r.ContiguousBefore)
                    r.Received.Add(sn);
            Advance(r);
        }
    }

    // Slides the contiguous frontier forward over out-of-order arrivals, freeing set entries.
    private static void Advance(Remote r)
    {
        while (r.Received.Remove(r.ContiguousBefore)) r.ContiguousBefore++;
    }
}
