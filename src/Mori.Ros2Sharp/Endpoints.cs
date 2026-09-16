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
        // The submessage counts last acted on: a repeat (the same datagram arriving through
        // several of our locators, or a retransmit) must not trigger another resend.
        public long LastAckNackCount = -1;
        public long LastNackFragCount = -1;
        // Set once the reader has answered anything: proof it has matched us, not just the
        // other way round. Best-effort readers never answer, so they count as confirmed
        // from the start.
        public bool Confirmed;
        // The first sequence number this reader is entitled to. A volatile reader gets
        // nothing written before it matched: requests for older samples are answered with
        // a GAP, never a resend. Transient-local readers of a transient-local writer start
        // at 1 and receive the replayed history.
        public long RelevantFrom = 1;
    }

    /// <summary>
    /// How many matched readers have confirmed the match from their side, by answering a
    /// HEARTBEAT. Discovery is symmetric but not simultaneous: a writer can have matched a
    /// reader whose own participant has not yet matched the writer, and samples sent in
    /// that window are dropped by a volatile reader. Best-effort readers never answer and
    /// count as confirmed as soon as they are matched.
    /// </summary>
    public int ConfirmedReaderCount { get { lock (_lock) return _readers.Count(r => r.Confirmed); } }

    /// <summary>
    /// Waits until at least <paramref name="count"/> readers have confirmed the match (see
    /// <see cref="ConfirmedReaderCount"/>), so a sample written afterwards reaches them. Returns
    /// false on timeout. The standard idiom for publish-once programs and the first message
    /// of a stream.
    /// </summary>
    public async Task<bool> WaitForReadersAsync(int count, TimeSpan timeout)
    {
        var deadline = DateTimeOffset.UtcNow + timeout;
        while (true)
        {
            if (ConfirmedReaderCount >= count) return true;
            if (DateTimeOffset.UtcNow >= deadline) return false;
            await Task.Delay(20);
        }
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

    /// <summary>
    /// Blocks until every matched reader has acknowledged every sample written so far, or the
    /// timeout passes. Call before disposing a reliable writer so retransmissions still under
    /// way can finish; returns false on timeout (or always true for a best-effort writer).
    /// </summary>
    public async Task<bool> WaitForAcknowledgmentsAsync(TimeSpan timeout)
    {
        if (!Reliable) return true;
        var deadline = DateTimeOffset.UtcNow + timeout;
        while (true)
        {
            lock (_lock)
                if (_readers.All(r => r.AckedBefore >= _nextSn)) return true;
            if (DateTimeOffset.UtcNow >= deadline) return false;
            await Task.Delay(20);
        }
    }

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
    /// Registers (or re-locates) a remote reader. When both sides are transient-local the
    /// new match gets the whole history replayed — that is how SEDP data reaches late-joining
    /// participants, and how a latched topic reaches a late subscription. A volatile reader
    /// of a transient-local writer gets only what is written from now on, as DDS specifies.
    /// </summary>
    internal void MatchReader(RtpsGuid readerGuid, IReadOnlyList<IPEndPoint> locators, bool readerTransientLocal)
    {
        Remote fresh;
        List<Sample>? replay = null;
        lock (_lock)
        {
            var existing = _readers.FirstOrDefault(r => r.Guid == readerGuid);
            if (existing != null) { existing.Locators = locators; return; }
            bool history = TransientLocal && readerTransientLocal;
            fresh = new Remote
            {
                Guid = readerGuid,
                Locators = locators,
                Confirmed = !Reliable,
                RelevantFrom = history ? 1 : _nextSn,
                AckedBefore = history ? 1 : _nextSn,
            };
            _readers.Add(fresh);
            if (history) replay = _history.ToList();
        }
        if (replay != null)
            foreach (var s in replay)
                SendData(fresh, s);
        if (Reliable) SendHeartbeat(fresh);
    }

    /// <summary>
    /// A HEARTBEAT to one reader regardless of whether it has anything to acknowledge. A
    /// non-final heartbeat draws an ACKNACK, which is how the reader confirms the match.
    /// With an empty history the advertised range is [1, 0], the RTPS "nothing yet" form.
    /// </summary>
    private void SendHeartbeat(Remote reader)
    {
        long first, last;
        uint count;
        lock (_lock)
        {
            // The range starts where this reader's entitlement does, so a volatile reader
            // never learns of (and never requests) samples from before its match.
            last = _history.Count > 0 ? _history[^1].Sn : _nextSn - 1;
            first = Math.Max(_history.Count > 0 ? _history[0].Sn : _nextSn, reader.RelevantFrom);
            if (first > last + 1) first = last + 1;
            count = ++_heartbeatCount;
        }
        _participant.SendDirected(reader.Guid.Prefix, reader.Locators,
            w => w.AddHeartbeat(reader.Guid.Entity, Guid.Entity, first, last, count, final: false));
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
            if (reader == null || an.Count <= reader.LastAckNackCount) return;
            reader.LastAckNackCount = an.Count;
            reader.Confirmed = true;
            reader.AckedBefore = Math.Max(reader.AckedBefore, an.BaseSn);
            // Below this the reader gets a GAP: history no longer holds it, or the reader
            // is volatile and the sample predates its match.
            firstAvailable = Math.Max(_history.Count > 0 ? _history[0].Sn : _nextSn, reader.RelevantFrom);
            foreach (long sn in an.Missing)
            {
                if (sn < firstAvailable) continue;
                var s = _history.FirstOrDefault(x => x.Sn == sn);
                if (s != null) resend.Add(s);
            }
        }
        foreach (var s in resend) SendData(reader, s);
        if (an.Missing.Any(sn => sn < firstAvailable))
            _participant.SendDirected(readerGuid.Prefix, reader.Locators,
                w => w.AddGap(readerGuid.Entity, Guid.Entity, an.BaseSn, firstAvailable));
    }

    /// <summary>
    /// One reliability tick: HEARTBEAT every matched reader that has not acked everything,
    /// plus every reader that has not yet confirmed the match (see <see cref="ConfirmedReaderCount"/>).
    /// </summary>
    internal void SendHeartbeats()
    {
        if (!Reliable) return;
        List<Remote> pending;
        lock (_lock)
        {
            long last = _history.Count > 0 ? _history[^1].Sn : 0;
            pending = _readers.Where(r => !r.Confirmed || (_history.Count > 0 && r.AckedBefore <= last)).ToList();
        }
        foreach (var r in pending) SendHeartbeat(r);
    }

    /// <summary>
    /// The reader still lacks some fragments of one sample: resend just those, or GAP the
    /// sample if history no longer holds it.
    /// </summary>
    internal void OnNackFrag(GuidPrefix source, RtpsNackFrag nf)
    {
        if (nf.WriterId != Guid.Entity) return;
        var readerGuid = new RtpsGuid(source, nf.ReaderId);
        Remote? reader;
        Sample? s;
        bool gone;
        lock (_lock)
        {
            reader = _readers.FirstOrDefault(r => r.Guid == readerGuid);
            if (reader == null || nf.Count <= reader.LastNackFragCount) return;
            reader.LastNackFragCount = nf.Count;
            reader.Confirmed = true;
            s = _history.FirstOrDefault(x => x.Sn == nf.SequenceNumber);
            gone = s == null && nf.SequenceNumber < _nextSn;
        }
        if (s != null && nf.Missing.Count > 0)
        {
            SendFragments(reader, s, nf.Missing);
            SendHeartbeats(); // readers ack (or ask again) on a HEARTBEAT, so do not make them wait for the periodic one
        }
        else if (gone)
            _participant.SendDirected(readerGuid.Prefix, reader.Locators,
                w => w.AddGap(readerGuid.Entity, Guid.Entity, nf.SequenceNumber, nf.SequenceNumber + 1));
    }

    // One DATA when the sample fits a datagram, DATA_FRAG runs otherwise.
    private void SendData(Remote reader, Sample s)
    {
        if (s.Payload.Length > _participant.MaxUnfragmentedPayload)
        {
            SendFragments(reader, s, null);
            return;
        }
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
    /// Pushes the listed fragments (all of them when <paramref name="only"/> is null) as
    /// DATA_FRAG runs, packing consecutive fragments into one datagram while they fit. From a
    /// reliable writer every push ends with a HEARTBEAT_FRAG so the reader can NACK_FRAG holes
    /// right away instead of waiting for the periodic HEARTBEAT.
    /// </summary>
    private void SendFragments(Remote reader, Sample s, IReadOnlyList<uint>? only)
    {
        int fragSize = _participant.FragmentSize;
        uint total = (uint)((s.Payload.Length + fragSize - 1) / fragSize);
        int perDatagram = Math.Max(1, (RtpsParticipant.MaxDatagramSize - RtpsParticipant.DatagramOverhead) / fragSize);
        IEnumerable<uint> wanted = only == null
            ? Enumerable.Range(1, (int)total).Select(i => (uint)i)
            : only.Where(f => f >= 1 && f <= total).Distinct().OrderBy(f => f);

        var run = new List<uint>();
        foreach (uint f in wanted)
        {
            if (run.Count > 0 && (f != run[^1] + 1 || run.Count == perDatagram)) Flush();
            run.Add(f);
        }
        Flush();

        // A HEARTBEAT_FRAG closes every push, initial or resend: it tells the reader the
        // fragments are all out, so a hole it still sees is a real loss to ask for again.
        if (Reliable)
        {
            uint count;
            lock (_lock) count = ++_heartbeatCount;
            _participant.SendDirected(reader.Guid.Prefix, reader.Locators,
                w => w.AddHeartbeatFrag(reader.Guid.Entity, Guid.Entity, s.Sn, total, count));
        }

        void Flush()
        {
            if (run.Count == 0) return;
            uint startNum = run[0];
            int count = run.Count;
            int offset = (int)((startNum - 1) * (long)fragSize);
            int len = (int)Math.Min((long)count * fragSize, s.Payload.Length - offset);
            _participant.SendDirected(reader.Guid.Prefix, reader.Locators, w =>
            {
                w.AddInfoTimestamp(s.Timestamp);
                w.AddDataFrag(reader.Guid.Entity, Guid.Entity, s.Sn, startNum, (ushort)count, (ushort)fragSize,
                    (uint)s.Payload.Length, s.Payload.AsSpan(offset, len), s.RelatedGuid, s.RelatedSn);
            });
            run.Clear();
        }
    }
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
    public bool TransientLocal { get; }

    private Action<RtpsGuid, byte[], RtpsTime?>? _dataReceived;
    private Action<RtpsGuid, RtpsData>? _sampleReceived;

    // Samples that arrived before any handler was attached. A transient-local writer replays
    // its history the instant the match completes, which can be before the line after
    // CreateSubscription runs; the first handler attached receives these in order.
    private readonly Queue<(RtpsGuid Writer, RtpsData Sample)> _held = new();
    private const int MaxHeld = 64;

    /// <summary>
    /// Raised once per new sample: writer guid, serialized payload, source timestamp. Samples
    /// received before any handler is attached are held (the most recent 64) and delivered to
    /// the first handler attached, so latched history is never missed by attaching late.
    /// </summary>
    public event Action<RtpsGuid, byte[], RtpsTime?>? DataReceived
    {
        add { lock (_deliverLock) { _dataReceived += value; FlushHeld(); } }
        remove { lock (_deliverLock) _dataReceived -= value; }
    }

    /// <summary>Raised once per new sample with full submessage detail (services use the identities). Same hold-back rule as <see cref="DataReceived"/>.</summary>
    public event Action<RtpsGuid, RtpsData>? SampleReceived
    {
        add { lock (_deliverLock) { _sampleReceived += value; FlushHeld(); } }
        remove { lock (_deliverLock) _sampleReceived -= value; }
    }

    // Under _deliverLock: hands every held sample to whatever handlers exist now.
    private void FlushHeld()
    {
        while (_held.Count > 0)
        {
            var (writer, d) = _held.Dequeue();
            _dataReceived?.Invoke(writer, d.Payload, d.Timestamp);
            _sampleReceived?.Invoke(writer, d);
        }
    }

    public int MatchedWriterCount { get { lock (_lock) return _writers.Count; } }

    // Bounds on fragment reassembly: how many half-built samples one writer may hold, the
    // largest sample we will allocate for, and how long a sample may go without a new
    // fragment before the ACKNACK path asks for the whole thing again.
    private const int MaxPendingPerWriter = 8;
    private const uint MaxSampleSize = 256 << 20;
    private static readonly TimeSpan StallTimeout = TimeSpan.FromSeconds(1);

    // A hole is not re-requested within this window of the previous NACK_FRAG for the same
    // sample: the resend is most likely still in flight, and every request draws another
    // HEARTBEAT from the writer, which without this turns into a request/heartbeat storm.
    private static readonly TimeSpan NackFragSuppression = TimeSpan.FromMilliseconds(50);

    // A plain HEARTBEAT says nothing about how many fragments the writer has pushed, and
    // writers piggyback them in the middle of a burst. Until a sample has been quiet this
    // long, only holes below the highest fragment seen are treated as lost; the tail is
    // presumed to be still on its way. Cyclone DDS sends 686 datagrams per 921 KB sample,
    // and requesting the in-flight tail turned every burst into a retransmission storm.
    // Short, because Cyclone's writer also stalls a large sample after its first fragment
    // (adaptive history watermark) and only the reader's request for the tail moves it on.
    private static readonly TimeSpan BurstQuiet = TimeSpan.FromMilliseconds(30);

    // How many 256-fragment NACK_FRAG windows one heartbeat response may carry per sample.
    private const int MaxNackFragWindows = 4;

    // Fast DDS piggybacks a non-final HEARTBEAT on nearly every fragment datagram, and Cyclone
    // answers every ACKNACK that requests something with another HEARTBEAT. An ACKNACK that
    // says exactly what the previous one said, within this window, only feeds that loop.
    private static readonly TimeSpan AckNackCoalesce = TimeSpan.FromMilliseconds(100);

    // A reliable reader delivers in sequence order, so a complete sample waits behind an
    // earlier hole. Past this many waiting samples the hole is presumed permanent (a writer
    // that died mid-stream) and delivery skips over it.
    private const int MaxReadyPerWriter = 32;

    private readonly object _deliverLock = new();

    private sealed class Remote
    {
        public IReadOnlyList<IPEndPoint> Locators = Array.Empty<IPEndPoint>();
        public long ContiguousBefore = 1;               // every sn below this was seen or gapped
        public readonly HashSet<long> Received = new(); // sns at/above ContiguousBefore, out of order
        public readonly Dictionary<long, Reassembly> Pending = new(); // fragmented samples under way
        public readonly SortedDictionary<long, RtpsData> Ready = new(); // complete, waiting for order
        public uint AckNackCount;
        public DateTimeOffset LastAckNack = DateTimeOffset.MinValue; // when, and with what content,
        public long LastAckNackBase = -1;                            // the last ACKNACK went out
        public List<long> LastAckNackMissing = new();
        public uint NackFragCount;   // one sequence per writer, as the writer's duplicate check expects
        public long LastHeartbeatCount = -1;     // repeats of a HEARTBEAT / HEARTBEAT_FRAG are ignored
        public long LastHeartbeatFragCount = -1;
    }

    /// <summary>One large sample being rebuilt from DATA_FRAG runs.</summary>
    private sealed class Reassembly
    {
        public readonly byte[] Buffer;
        public readonly bool[] Have;
        public readonly int FragmentSize;
        public int Remaining;
        public uint HighestReceived;   // 1-based number of the highest fragment seen so far
        public DateTimeOffset LastActivity;
        public DateTimeOffset LastNackFrag = DateTimeOffset.MinValue;
        public RtpsTime? Timestamp;
        public byte[]? KeyHash;
        public bool Disposed;
        public RtpsGuid? RelatedGuid;
        public long RelatedSn;

        public Reassembly(uint sampleSize, int fragmentSize)
        {
            Buffer = new byte[sampleSize];
            FragmentSize = fragmentSize;
            Have = new bool[(sampleSize + (uint)fragmentSize - 1) / (uint)fragmentSize];
            Remaining = Have.Length;
        }

        /// <summary>Copies the fragments a submessage carries into place; duplicates are ignored.</summary>
        public void Absorb(RtpsDataFrag f, DateTimeOffset now)
        {
            for (int i = 0; i < f.FragmentsInSubmessage; i++)
            {
                long idx = f.FragmentStartingNum - 1 + i;
                int src = i * FragmentSize;
                if (idx >= Have.Length || src >= f.Fragments.Length) break;
                int dst = (int)(idx * FragmentSize);
                int len = Math.Min(FragmentSize, Buffer.Length - dst);
                if (src + len > f.Fragments.Length) break;
                if (Have[idx]) continue;
                f.Fragments.AsSpan(src, len).CopyTo(Buffer.AsSpan(dst));
                Have[idx] = true;
                Remaining--;
                HighestReceived = Math.Max(HighestReceived, (uint)idx + 1);
            }
            LastActivity = now;
            Timestamp ??= f.Timestamp;
            KeyHash ??= f.KeyHash;
            Disposed |= f.Disposed;
            if (f.RelatedGuid is { } g) { RelatedGuid = g; RelatedSn = f.RelatedSn; }
        }

        /// <summary>
        /// The missing fragment numbers from <paramref name="from"/> up to <paramref name="upTo"/>
        /// (both 1-based, inclusive) within one NACK_FRAG window: 256 from the first hole.
        /// Empty when nothing is missing there.
        /// </summary>
        public List<uint> Missing(uint from, uint upTo, out uint baseFragment)
        {
            var list = new List<uint>();
            baseFragment = 0;
            long limit = Math.Min(upTo, (uint)Have.Length);
            for (long i = Math.Max(0, (long)from - 1); i < limit; i++)
            {
                if (Have[i]) continue;
                uint n = (uint)i + 1;
                if (list.Count == 0) baseFragment = n;
                else if (n - baseFragment >= 256) break;
                list.Add(n);
            }
            return list;
        }

        public RtpsData ToData(RtpsDataFrag last) => new()
        {
            ReaderId = last.ReaderId,
            WriterId = last.WriterId,
            SequenceNumber = last.SequenceNumber,
            Timestamp = Timestamp,
            Payload = Buffer,
            KeyHash = KeyHash,
            Disposed = Disposed,
            RelatedGuid = RelatedGuid,
            RelatedSn = RelatedSn,
        };
    }

    internal RtpsReaderEndpoint(RtpsParticipant participant, RtpsGuid guid, string topicName,
        string typeName, bool reliable, bool transientLocal = false)
    {
        _participant = participant;
        Guid = guid;
        TopicName = topicName;
        TypeName = typeName;
        Reliable = reliable;
        TransientLocal = transientLocal;
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
        if (d.ReaderId != EntityId.Unknown && d.ReaderId != Guid.Entity) return false; // addressed elsewhere
        var writerGuid = new RtpsGuid(source, d.WriterId);
        var deliver = new List<RtpsData>();
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return false;
            if (d.Payload.Length == 0) return true; // dispose/unregister — endpoint removal, later
            if (d.SequenceNumber < r.ContiguousBefore || !r.Received.Add(d.SequenceNumber)) return true;
            Advance(r);
            Accept(r, d, deliver);
        }
        Deliver(writerGuid, deliver);
        return true;
    }

    // A complete sample: best-effort readers hand it over at once, reliable readers only
    // once everything before it has been delivered or declared unavailable.
    private void Accept(Remote r, RtpsData d, List<RtpsData> deliver)
    {
        if (!Reliable) { deliver.Add(d); return; }
        r.Ready[d.SequenceNumber] = d;
        Drain(r, deliver);
    }

    // Moves every ready sample below the contiguous frontier into the delivery list, skipping
    // a hole outright when too many samples have piled up behind it.
    private static void Drain(Remote r, List<RtpsData> deliver)
    {
        while (true)
        {
            while (r.Ready.Count > 0 && r.Ready.First().Key < r.ContiguousBefore)
            {
                var first = r.Ready.First();
                deliver.Add(first.Value);
                r.Ready.Remove(first.Key);
            }
            if (r.Ready.Count <= MaxReadyPerWriter) return;
            long skipTo = r.Ready.First().Key;
            r.ContiguousBefore = skipTo;
            r.Received.RemoveWhere(sn => sn < skipTo);
            foreach (long sn in r.Pending.Keys.Where(sn => sn < skipTo).ToList())
                r.Pending.Remove(sn);
            Advance(r);
        }
    }

    // Raises the events in order; the lock keeps deliveries from the three receive loops
    // from interleaving within one reader.
    private void Deliver(RtpsGuid writerGuid, List<RtpsData> samples)
    {
        if (samples.Count == 0) return;
        lock (_deliverLock)
            foreach (var d in samples)
            {
                if (_dataReceived == null && _sampleReceived == null)
                {
                    if (_held.Count == MaxHeld) _held.Dequeue();
                    _held.Enqueue((writerGuid, d));
                    continue;
                }
                _dataReceived?.Invoke(writerGuid, d.Payload, d.Timestamp);
                _sampleReceived?.Invoke(writerGuid, d);
            }
    }

    /// <summary>
    /// Accepts a run of fragments from a matched writer, delivering the sample once every
    /// fragment is in. Returns true when the fragments belonged to one of our writers.
    /// </summary>
    internal bool OnDataFrag(GuidPrefix source, RtpsDataFrag f)
    {
        if (f.ReaderId != EntityId.Unknown && f.ReaderId != Guid.Entity) return false; // addressed elsewhere
        var writerGuid = new RtpsGuid(source, f.WriterId);
        var deliver = new List<RtpsData>();
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return false;
            if (f.SequenceNumber < r.ContiguousBefore || r.Received.Contains(f.SequenceNumber)) return true;
            if (f.SampleSize > MaxSampleSize) return true;
            if (!r.Pending.TryGetValue(f.SequenceNumber, out var a))
            {
                if (r.Pending.Count >= MaxPendingPerWriter) r.Pending.Remove(r.Pending.Keys.Min());
                a = new Reassembly(f.SampleSize, f.FragmentSize);
                r.Pending[f.SequenceNumber] = a;
            }
            else if (a.FragmentSize != f.FragmentSize || a.Buffer.Length != f.SampleSize)
                return true; // not the same sample we started on; keep what we have
            a.Absorb(f, DateTimeOffset.UtcNow);
            if (a.Remaining > 0) return true;
            r.Pending.Remove(f.SequenceNumber);
            r.Received.Add(f.SequenceNumber);
            Advance(r);
            Accept(r, a.ToData(f), deliver);
        }
        Deliver(writerGuid, deliver);
        return true;
    }

    /// <summary>
    /// Answers a writer's HEARTBEAT with an ACKNACK: ack everything contiguous, request the
    /// holes up to LastSn. Silence is only allowed when nothing is missing and the heartbeat
    /// was final. Samples still arriving in fragments are asked for with NACK_FRAG instead
    /// of being listed as missing, so the writer resends only the holes; once such a sample
    /// stalls it goes back into the ACKNACK and the writer resends it whole.
    /// </summary>
    internal void OnHeartbeat(GuidPrefix source, RtpsHeartbeat hb)
    {
        if (hb.ReaderId != EntityId.Unknown && hb.ReaderId != Guid.Entity) return;
        var writerGuid = new RtpsGuid(source, hb.WriterId);
        long baseSn;
        var missing = new List<long>();
        var nackFrags = new List<(long Sn, uint Base, List<uint> Missing, uint Count)>();
        uint count = 0;
        IReadOnlyList<IPEndPoint> locators;
        var deliver = new List<RtpsData>();
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return;
            if (hb.Count <= r.LastHeartbeatCount) return;
            r.LastHeartbeatCount = hb.Count;
            if (r.ContiguousBefore < hb.FirstSn) // samples below FirstSn are no longer available
            {
                r.ContiguousBefore = hb.FirstSn;
                r.Received.RemoveWhere(sn => sn < hb.FirstSn);
                foreach (long sn in r.Pending.Keys.Where(sn => sn < hb.FirstSn).ToList())
                    r.Pending.Remove(sn);
                Advance(r);
                Drain(r, deliver);
            }
            baseSn = r.ContiguousBefore;
            var now = DateTimeOffset.UtcNow;
            for (long sn = baseSn; sn <= hb.LastSn && missing.Count < 256; sn++)
            {
                if (r.Received.Contains(sn)) continue;
                if (r.Pending.TryGetValue(sn, out var a) && now - a.LastActivity < StallTimeout)
                {
                    if (now - a.LastNackFrag < NackFragSuppression) continue; // asked moments ago
                    bool quiet = now - a.LastActivity >= BurstQuiet;
                    // While fragments keep arriving, a request is worthwhile once (a hole in
                    // the initial burst); after that the resend under way must finish first.
                    if (!quiet && a.LastNackFrag != DateTimeOffset.MinValue) continue;
                    // Mid-burst, only holes below the highest fragment seen are real losses.
                    uint upTo = quiet ? uint.MaxValue : a.HighestReceived;
                    uint from = 1;
                    for (int w = 0; w < MaxNackFragWindows; w++)
                    {
                        var holes = a.Missing(from, upTo, out uint baseFrag);
                        if (holes.Count == 0) break;
                        nackFrags.Add((sn, baseFrag, holes, ++r.NackFragCount));
                        a.LastNackFrag = now;
                        from = baseFrag + 256;
                    }
                    continue;
                }
                missing.Add(sn);
            }
            bool changed = baseSn != r.LastAckNackBase || !missing.SequenceEqual(r.LastAckNackMissing);
            bool ackNack = changed || now - r.LastAckNack >= AckNackCoalesce;
            if (!ackNack && nackFrags.Count == 0) { Deliver(writerGuid, deliver); return; }
            if (ackNack)
            {
                count = ++r.AckNackCount;
                r.LastAckNack = now;
                r.LastAckNackBase = baseSn;
                r.LastAckNackMissing = missing;
            }
            locators = r.Locators;
        }
        Deliver(writerGuid, deliver);
        bool sendAckNack = count != 0;
        _participant.SendDirected(source, locators, w =>
        {
            if (sendAckNack)
                w.AddAckNack(Guid.Entity, hb.WriterId, baseSn, missing, count, final: missing.Count == 0);
            foreach (var nf in nackFrags)
                w.AddNackFrag(Guid.Entity, hb.WriterId, nf.Sn, nf.Base, nf.Missing, nf.Count);
        });
    }

    /// <summary>
    /// A writer finished pushing fragments of one sample: ask for whatever did not arrive.
    /// Unlike a plain HEARTBEAT this carries the fragment count, so it is answered on its own
    /// count alone — the suppression window only guards against re-requesting in-flight data.
    /// </summary>
    internal void OnHeartbeatFrag(GuidPrefix source, RtpsHeartbeatFrag hf)
    {
        if (hf.ReaderId != EntityId.Unknown && hf.ReaderId != Guid.Entity) return;
        var writerGuid = new RtpsGuid(source, hf.WriterId);
        List<uint> holes;
        uint baseFrag, count;
        IReadOnlyList<IPEndPoint> locators;
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return;
            if (hf.Count <= r.LastHeartbeatFragCount) return;
            r.LastHeartbeatFragCount = hf.Count;
            if (!r.Pending.TryGetValue(hf.SequenceNumber, out var a)) return; // nothing yet, or already complete
            holes = a.Missing(1, hf.LastFragmentNum, out baseFrag);
            if (holes.Count == 0) return;
            a.LastNackFrag = DateTimeOffset.UtcNow;
            count = ++r.NackFragCount;
            locators = r.Locators;
        }
        _participant.SendDirected(source, locators,
            w => w.AddNackFrag(Guid.Entity, hf.WriterId, hf.SequenceNumber, baseFrag, holes, count));
    }

    /// <summary>Marks GAP-listed sequence numbers as seen so we stop requesting them.</summary>
    internal void OnGap(GuidPrefix source, RtpsGap gap)
    {
        if (gap.ReaderId != EntityId.Unknown && gap.ReaderId != Guid.Entity) return;
        var writerGuid = new RtpsGuid(source, gap.WriterId);
        var deliver = new List<RtpsData>();
        lock (_lock)
        {
            if (!_writers.TryGetValue(writerGuid, out var r)) return;
            for (long sn = gap.GapStart; sn < gap.GapListBase; sn++)
                if (sn >= r.ContiguousBefore)
                { r.Received.Add(sn); r.Pending.Remove(sn); }
            foreach (long sn in gap.Bits)
                if (sn >= r.ContiguousBefore)
                { r.Received.Add(sn); r.Pending.Remove(sn); }
            Advance(r);
            Drain(r, deliver);
        }
        Deliver(writerGuid, deliver);
    }

    // Slides the contiguous frontier forward over out-of-order arrivals, freeing set entries.
    private static void Advance(Remote r)
    {
        while (r.Received.Remove(r.ContiguousBefore)) r.ContiguousBefore++;
    }
}
