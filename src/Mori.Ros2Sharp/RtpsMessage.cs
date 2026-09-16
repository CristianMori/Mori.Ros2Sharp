using System.Buffers.Binary;

namespace Mori.Ros2Sharp;

/// <summary>Builds one RTPS datagram: the fixed 20-byte header followed by submessages.</summary>
public sealed class RtpsMessageWriter
{
    private byte[] _b = new byte[512];
    private int _len;

    public RtpsMessageWriter(GuidPrefix source)
    {
        _b[0] = (byte)'R'; _b[1] = (byte)'T'; _b[2] = (byte)'P'; _b[3] = (byte)'S';
        _b[4] = ProtocolVersion.Local.Major;
        _b[5] = ProtocolVersion.Local.Minor;
        _b[6] = VendorId.Local.Hi;
        _b[7] = VendorId.Local.Lo;
        source.WriteTo(_b.AsSpan(8));
        _len = 20;
    }

    /// <summary>Stamps the source time onto the submessages that follow.</summary>
    public void AddInfoTimestamp(RtpsTime time)
    {
        int start = Begin(0x09, 0x01);
        I32(time.Seconds);
        U32(time.Fraction);
        End(start);
    }

    /// <summary>Scopes the following submessages to one destination participant.</summary>
    public void AddInfoDestination(GuidPrefix destination)
    {
        int start = Begin(0x0e, 0x01);
        Ensure(12);
        destination.WriteTo(_b.AsSpan(_len));
        _len += 12;
        End(start);
    }

    /// <summary>One sample from a writer: entity ids, sequence number, serialized payload.</summary>
    public void AddData(EntityId readerId, EntityId writerId, long sequenceNumber, ReadOnlySpan<byte> serializedPayload)
    {
        int start = Begin(0x15, 0x05); // little-endian + serialized data present
        U16(0);                        // extraFlags
        U16(16);                       // octetsToInlineQos: readerId + writerId + sequence number
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        Bytes(serializedPayload);
        End(start);
    }

    /// <summary>A DATA whose inline QoS carries a related sample identity (service correlation).</summary>
    public void AddData(EntityId readerId, EntityId writerId, long sequenceNumber,
        ReadOnlySpan<byte> serializedPayload, RtpsGuid relatedGuid, long relatedSn)
    {
        int start = Begin(0x15, 0x07); // little-endian + inline QoS + serialized data
        U16(0);
        U16(16);
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        Span<byte> g = stackalloc byte[16];
        relatedGuid.WriteTo(g);
        U16(Pid.RelatedSampleIdentity); U16(24); Bytes(g); Sn(relatedSn);
        U16(Pid.Sentinel); U16(0);
        Bytes(serializedPayload);
        End(start);
    }

    /// <summary>
    /// A DATA carrying no payload — just the instance key and a disposed/unregistered status in
    /// inline QoS. How a writer announces that an instance (e.g. a participant) went away.
    /// </summary>
    public void AddDisposeData(EntityId readerId, EntityId writerId, long sequenceNumber, RtpsGuid keyHash)
    {
        int start = Begin(0x15, 0x03); // little-endian + inline QoS, no serialized payload
        U16(0);
        U16(16);
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        Span<byte> kh = stackalloc byte[16];
        keyHash.WriteTo(kh);
        U16(Pid.KeyHash); U16(16); Bytes(kh);
        U16(Pid.StatusInfo); U16(4); Bytes(stackalloc byte[] { 0, 0, 0, 0x03 }); // disposed + unregistered
        U16(Pid.Sentinel); U16(0);
        End(start);
    }

    /// <summary>
    /// A run of consecutive fragments of one sample. All fragments of a sample share one
    /// fragment size; only the sample's last fragment may be shorter. Fragment numbers are
    /// 1-based. Note the flag layout differs from DATA: here 0x04 is the key flag and there
    /// is no "data present" bit — a DATA_FRAG always carries payload.
    /// </summary>
    public void AddDataFrag(EntityId readerId, EntityId writerId, long sequenceNumber,
        uint fragmentStartingNum, ushort fragmentsInSubmessage, ushort fragmentSize, uint sampleSize,
        ReadOnlySpan<byte> fragmentBytes, RtpsGuid? relatedGuid = null, long relatedSn = 0)
    {
        int start = Begin(0x16, (byte)(relatedGuid is null ? 0x01 : 0x03)); // little-endian [+ inline QoS]
        U16(0);                        // extraFlags
        U16(28);                       // octetsToInlineQos: ids + sn + fragment header
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        U32(fragmentStartingNum);
        U16(fragmentsInSubmessage);
        U16(fragmentSize);
        U32(sampleSize);
        if (relatedGuid is { } rg)
        {
            Span<byte> g = stackalloc byte[16];
            rg.WriteTo(g);
            U16(Pid.RelatedSampleIdentity); U16(24); Bytes(g); Sn(relatedSn);
            U16(Pid.Sentinel); U16(0);
        }
        Bytes(fragmentBytes);
        End(start);
    }

    /// <summary>Tells the reader which fragments of one sample the writer has sent so far.</summary>
    public void AddHeartbeatFrag(EntityId readerId, EntityId writerId, long sequenceNumber, uint lastFragmentNum, uint count)
    {
        int start = Begin(0x13, 0x01);
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        U32(lastFragmentNum);
        U32(count);
        End(start);
    }

    /// <summary>
    /// Requests specific fragments of one sample. The fragment-number set is 32-bit based
    /// (unlike the 64-bit sequence-number set of ACKNACK) and covers at most 256 fragments
    /// from <paramref name="baseFragment"/>.
    /// </summary>
    public void AddNackFrag(EntityId readerId, EntityId writerId, long sequenceNumber,
        uint baseFragment, IReadOnlyList<uint> missing, uint count)
    {
        int start = Begin(0x12, 0x01);
        Entity(readerId);
        Entity(writerId);
        Sn(sequenceNumber);
        U32(baseFragment);
        int numBits = 0;
        foreach (uint f in missing)
            if (f >= baseFragment && f - baseFragment < 256)
                numBits = Math.Max(numBits, (int)(f - baseFragment) + 1);
        Span<uint> bitmap = stackalloc uint[8];
        foreach (uint f in missing)
        {
            long i = (long)f - baseFragment;
            if (i >= 0 && i < numBits) bitmap[(int)(i / 32)] |= 1u << (31 - (int)(i % 32)); // MSB-first
        }
        U32((uint)numBits);
        for (int i = 0; i < (numBits + 31) / 32; i++) U32(bitmap[i]);
        U32(count);
        End(start);
    }

    /// <summary>
    /// Advertises the writer's available range [firstSn, lastSn]. A non-final heartbeat obliges
    /// the reader to respond with an ACKNACK even when it is missing nothing.
    /// </summary>
    public void AddHeartbeat(EntityId readerId, EntityId writerId, long firstSn, long lastSn, uint count, bool final)
    {
        int start = Begin(0x07, (byte)(final ? 0x03 : 0x01));
        Entity(readerId);
        Entity(writerId);
        Sn(firstSn);
        Sn(lastSn);
        U32(count);
        End(start);
    }

    /// <summary>Acknowledges everything below <paramref name="baseSn"/> and requests the given samples.</summary>
    public void AddAckNack(EntityId readerId, EntityId writerId, long baseSn, IReadOnlyList<long> missing, uint count, bool final)
    {
        int start = Begin(0x06, (byte)(final ? 0x03 : 0x01));
        Entity(readerId);
        Entity(writerId);
        Sn(baseSn);
        int numBits = 0;
        foreach (long sn in missing)
            if (sn >= baseSn && sn - baseSn < 256)
                numBits = Math.Max(numBits, (int)(sn - baseSn) + 1);
        Span<uint> bitmap = stackalloc uint[8];
        foreach (long sn in missing)
        {
            long i = sn - baseSn;
            if (i >= 0 && i < numBits) bitmap[(int)(i / 32)] |= 1u << (31 - (int)(i % 32)); // MSB-first
        }
        U32((uint)numBits);
        for (int i = 0; i < (numBits + 31) / 32; i++) U32(bitmap[i]);
        U32(count);
        End(start);
    }

    /// <summary>Declares [gapStart, gapListBase) as irrelevant — the reader must not wait for them.</summary>
    public void AddGap(EntityId readerId, EntityId writerId, long gapStart, long gapListBase)
    {
        int start = Begin(0x08, 0x01);
        Entity(readerId);
        Entity(writerId);
        Sn(gapStart);
        Sn(gapListBase);
        U32(0); // empty trailing bitmap
        End(start);
    }

    /// <summary>The finished datagram, ready to hand to a socket.</summary>
    public byte[] ToArray() => _b.AsSpan(0, _len).ToArray();

    // Begin/End bracket one submessage: Begin writes id + flags and reserves the 16-bit
    // octetsToNextHeader slot; End pads the content to 4 bytes and backpatches the length.
    private int Begin(byte id, byte flags)
    {
        Ensure(4);
        _b[_len++] = id;
        _b[_len++] = flags;
        _len += 2; // octetsToNextHeader, backpatched by End
        return _len;
    }

    private void End(int contentStart)
    {
        while ((_len - contentStart) % 4 != 0) { Ensure(1); _b[_len++] = 0; } // next header must be 4-aligned
        BinaryPrimitives.WriteUInt16LittleEndian(_b.AsSpan(contentStart - 2), (ushort)(_len - contentStart));
    }

    private void Ensure(int more)
    {
        if (_len + more <= _b.Length) return;
        int size = _b.Length * 2;
        while (size < _len + more) size *= 2;
        Array.Resize(ref _b, size);
    }

    private void U16(ushort v) { Ensure(2); BinaryPrimitives.WriteUInt16LittleEndian(_b.AsSpan(_len), v); _len += 2; }
    private void U32(uint v) { Ensure(4); BinaryPrimitives.WriteUInt32LittleEndian(_b.AsSpan(_len), v); _len += 4; }
    private void I32(int v) { Ensure(4); BinaryPrimitives.WriteInt32LittleEndian(_b.AsSpan(_len), v); _len += 4; }
    private void Entity(EntityId id) { Ensure(4); id.WriteTo(_b.AsSpan(_len)); _len += 4; }
    private void Sn(long sn) { I32((int)(sn >> 32)); U32((uint)sn); }
    private void Bytes(ReadOnlySpan<byte> s) { Ensure(s.Length); s.CopyTo(_b.AsSpan(_len)); _len += s.Length; }
}

/// <summary>A DATA submessage lifted out of a parsed datagram.</summary>
public sealed class RtpsData
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long SequenceNumber { get; init; }

    /// <summary>The source timestamp from the preceding INFO_TS submessage, if any.</summary>
    public RtpsTime? Timestamp { get; init; }

    /// <summary>The serialized payload, starting at its encapsulation header (empty if none).</summary>
    public byte[] Payload { get; init; } = Array.Empty<byte>();

    /// <summary>The 16-byte instance key from inline QoS, when present.</summary>
    public byte[]? KeyHash { get; init; }

    /// <summary>True when inline QoS carried a disposed or unregistered status.</summary>
    public bool Disposed { get; init; }

    /// <summary>The related sample identity from inline QoS (service correlation), when present.</summary>
    public RtpsGuid? RelatedGuid { get; init; }
    public long RelatedSn { get; init; }
}

/// <summary>A DATA_FRAG submessage: a run of consecutive fragments of one large sample.</summary>
public sealed class RtpsDataFrag
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long SequenceNumber { get; init; }
    public RtpsTime? Timestamp { get; init; }

    /// <summary>1-based number of the first fragment carried here.</summary>
    public uint FragmentStartingNum { get; init; }

    /// <summary>How many consecutive fragments follow, starting at <see cref="FragmentStartingNum"/>.</summary>
    public int FragmentsInSubmessage { get; init; }

    /// <summary>The size every fragment of this sample uses (the sample's last one may be shorter).</summary>
    public int FragmentSize { get; init; }

    /// <summary>The full serialized size of the sample being reassembled, encapsulation included.</summary>
    public uint SampleSize { get; init; }

    /// <summary>The fragment bytes, trimmed to the sample's end; padding is never included.</summary>
    public byte[] Fragments { get; init; } = Array.Empty<byte>();

    public byte[]? KeyHash { get; init; }
    public bool Disposed { get; init; }
    public RtpsGuid? RelatedGuid { get; init; }
    public long RelatedSn { get; init; }
}

/// <summary>A HEARTBEAT_FRAG submessage: the writer has sent fragments 1..LastFragmentNum of one sample.</summary>
public sealed class RtpsHeartbeatFrag
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long SequenceNumber { get; init; }
    public uint LastFragmentNum { get; init; }
    public uint Count { get; init; }
}

/// <summary>A NACK_FRAG submessage: the reader still needs the listed fragments of one sample.</summary>
public sealed class RtpsNackFrag
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long SequenceNumber { get; init; }
    public List<uint> Missing { get; init; } = new();
    public uint Count { get; init; }
}

/// <summary>A HEARTBEAT submessage: the writer's available sequence-number range.</summary>
public sealed class RtpsHeartbeat
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long FirstSn { get; init; }
    public long LastSn { get; init; }
    public uint Count { get; init; }
    public bool Final { get; init; }
}

/// <summary>An ACKNACK submessage: everything below Base is acknowledged, Missing is requested.</summary>
public sealed class RtpsAckNack
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long BaseSn { get; init; }
    public List<long> Missing { get; init; } = new();
    public uint Count { get; init; }
    public bool Final { get; init; }
}

/// <summary>A GAP submessage: [GapStart, GapListBase) plus Bits are irrelevant to the reader.</summary>
public sealed class RtpsGap
{
    public EntityId ReaderId { get; init; }
    public EntityId WriterId { get; init; }
    public long GapStart { get; init; }
    public long GapListBase { get; init; }
    public List<long> Bits { get; init; } = new();
}

/// <summary>A parsed RTPS datagram: the header fields plus the submessages we understand.</summary>
public sealed class RtpsMessage
{
    public ProtocolVersion Version { get; private init; }
    public VendorId Vendor { get; private init; }
    public GuidPrefix Source { get; private init; }

    /// <summary>The INFO_DST destination, when the datagram was scoped to one participant.</summary>
    public GuidPrefix? Destination { get; private set; }

    public List<RtpsData> Data { get; } = new();
    public List<RtpsDataFrag> DataFrags { get; } = new();
    public List<RtpsHeartbeat> Heartbeats { get; } = new();
    public List<RtpsHeartbeatFrag> HeartbeatFrags { get; } = new();
    public List<RtpsAckNack> AckNacks { get; } = new();
    public List<RtpsNackFrag> NackFrags { get; } = new();
    public List<RtpsGap> Gaps { get; } = new();

    /// <summary>
    /// Parses one datagram. Returns false only when it is not RTPS at all (bad magic, wrong
    /// major version, truncated header); unknown submessage kinds are skipped, not errors.
    /// </summary>
    public static bool TryParse(ReadOnlySpan<byte> datagram, out RtpsMessage? message)
    {
        message = null;
        if (datagram.Length < 20 ||
            datagram[0] != 'R' || datagram[1] != 'T' || datagram[2] != 'P' || datagram[3] != 'S' ||
            datagram[4] != 2)
            return false;

        var m = new RtpsMessage
        {
            Version = new ProtocolVersion(datagram[4], datagram[5]),
            Vendor = new VendorId(datagram[6], datagram[7]),
            Source = GuidPrefix.ReadFrom(datagram.Slice(8, 12)),
        };

        RtpsTime? timestamp = null;
        int at = 20;
        while (at + 4 <= datagram.Length)
        {
            byte id = datagram[at];
            byte flags = datagram[at + 1];
            bool le = (flags & 0x01) != 0;
            int octets = ReadU16(datagram.Slice(at + 2), le);
            int contentStart = at + 4;
            int contentLen = octets == 0 ? datagram.Length - contentStart : octets; // 0 = extends to the end
            if (contentStart + contentLen > datagram.Length) break;
            var content = datagram.Slice(contentStart, contentLen);

            switch (id)
            {
                case 0x09: // INFO_TS (flag 0x02 = invalidate)
                    if ((flags & 0x02) == 0 && content.Length >= 8)
                        timestamp = new RtpsTime(ReadI32(content, le), ReadU32(content.Slice(4), le));
                    break;
                case 0x0e: // INFO_DST
                    if (content.Length >= 12)
                        m.Destination = GuidPrefix.ReadFrom(content);
                    break;
                case 0x15: // DATA
                    if (TryParseData(content, flags, le, timestamp, out var data))
                        m.Data.Add(data);
                    break;
                case 0x16: // DATA_FRAG
                    if (TryParseDataFrag(content, flags, le, timestamp, out var frag))
                        m.DataFrags.Add(frag);
                    break;
                case 0x13 when content.Length >= 24: // HEARTBEAT_FRAG
                    m.HeartbeatFrags.Add(new RtpsHeartbeatFrag
                    {
                        ReaderId = EntityId.ReadFrom(content),
                        WriterId = EntityId.ReadFrom(content.Slice(4)),
                        SequenceNumber = ReadSn(content.Slice(8), le),
                        LastFragmentNum = ReadU32(content.Slice(16), le),
                        Count = ReadU32(content.Slice(20), le),
                    });
                    break;
                case 0x12 when content.Length >= 28: // NACK_FRAG
                {
                    uint baseFrag = ReadU32(content.Slice(16), le);
                    uint numBits = ReadU32(content.Slice(20), le);
                    int words = ((int)numBits + 31) / 32;
                    if (numBits > 256 || content.Length < 24 + words * 4 + 4) break;
                    var nf = new RtpsNackFrag
                    {
                        ReaderId = EntityId.ReadFrom(content),
                        WriterId = EntityId.ReadFrom(content.Slice(4)),
                        SequenceNumber = ReadSn(content.Slice(8), le),
                        Count = ReadU32(content.Slice(24 + words * 4), le),
                    };
                    for (int i = 0; i < numBits; i++)
                        if ((ReadU32(content.Slice(24 + (i / 32) * 4), le) & (1u << (31 - i % 32))) != 0)
                            nf.Missing.Add(baseFrag + (uint)i);
                    m.NackFrags.Add(nf);
                    break;
                }
                case 0x07 when content.Length >= 28: // HEARTBEAT
                    m.Heartbeats.Add(new RtpsHeartbeat
                    {
                        ReaderId = EntityId.ReadFrom(content),
                        WriterId = EntityId.ReadFrom(content.Slice(4)),
                        FirstSn = ReadSn(content.Slice(8), le),
                        LastSn = ReadSn(content.Slice(16), le),
                        Count = ReadU32(content.Slice(24), le),
                        Final = (flags & 0x02) != 0,
                    });
                    break;
                case 0x06 when content.Length >= 24: // ACKNACK
                {
                    long baseSn = ReadSn(content.Slice(8), le);
                    uint numBits = ReadU32(content.Slice(16), le);
                    int words = ((int)numBits + 31) / 32;
                    if (numBits > 256 || content.Length < 20 + words * 4 + 4) break;
                    var an = new RtpsAckNack
                    {
                        ReaderId = EntityId.ReadFrom(content),
                        WriterId = EntityId.ReadFrom(content.Slice(4)),
                        BaseSn = baseSn,
                        Count = ReadU32(content.Slice(20 + words * 4), le),
                        Final = (flags & 0x02) != 0,
                    };
                    for (int i = 0; i < numBits; i++)
                        if ((ReadU32(content.Slice(20 + (i / 32) * 4), le) & (1u << (31 - i % 32))) != 0)
                            an.Missing.Add(baseSn + i);
                    m.AckNacks.Add(an);
                    break;
                }
                case 0x08 when content.Length >= 28: // GAP
                {
                    long gapBase = ReadSn(content.Slice(16), le);
                    uint numBits = ReadU32(content.Slice(24), le);
                    int words = ((int)numBits + 31) / 32;
                    if (numBits > 256 || content.Length < 28 + words * 4) break;
                    var gap = new RtpsGap
                    {
                        ReaderId = EntityId.ReadFrom(content),
                        WriterId = EntityId.ReadFrom(content.Slice(4)),
                        GapStart = ReadSn(content.Slice(8), le),
                        GapListBase = gapBase,
                    };
                    for (int i = 0; i < numBits; i++)
                        if ((ReadU32(content.Slice(28 + (i / 32) * 4), le) & (1u << (31 - i % 32))) != 0)
                            gap.Bits.Add(gapBase + i);
                    m.Gaps.Add(gap);
                    break;
                }
            }

            if (octets == 0) break;
            at = contentStart + contentLen;
        }

        message = m;
        return true;
    }

    private static bool TryParseData(ReadOnlySpan<byte> c, byte flags, bool le, RtpsTime? timestamp, out RtpsData data)
    {
        data = null!;
        if (c.Length < 20) return false;

        int octetsToInlineQos = ReadU16(c.Slice(2), le);
        var readerId = EntityId.ReadFrom(c.Slice(4));
        var writerId = EntityId.ReadFrom(c.Slice(8));
        long sn = ((long)ReadI32(c.Slice(12), le) << 32) | ReadU32(c.Slice(16), le);

        int at = 4 + octetsToInlineQos;
        if (at > c.Length) return false;
        var qos = (flags & 0x02) != 0 ? ParseInlineQos(c, ref at, le) : default;

        byte[] payload = Array.Empty<byte>();
        if ((flags & 0x0c) != 0 && at <= c.Length) // data or key present
            payload = c.Slice(at).ToArray();

        data = new RtpsData
        {
            ReaderId = readerId,
            WriterId = writerId,
            SequenceNumber = sn,
            Timestamp = timestamp,
            Payload = payload,
            KeyHash = qos.KeyHash,
            Disposed = qos.Disposed,
            RelatedGuid = qos.RelatedGuid,
            RelatedSn = qos.RelatedSn,
        };
        return true;
    }

    // DATA_FRAG: the DATA prologue, then the fragment header (start number, count, size,
    // sample size), optional inline QoS, and the fragment bytes. Anything internally
    // inconsistent (zero sizes, a start past the sample, fewer bytes than announced) is
    // rejected rather than partially trusted.
    private static bool TryParseDataFrag(ReadOnlySpan<byte> c, byte flags, bool le, RtpsTime? timestamp, out RtpsDataFrag frag)
    {
        frag = null!;
        if (c.Length < 32) return false;

        int octetsToInlineQos = ReadU16(c.Slice(2), le);
        var readerId = EntityId.ReadFrom(c.Slice(4));
        var writerId = EntityId.ReadFrom(c.Slice(8));
        long sn = ReadSn(c.Slice(12), le);
        uint startNum = ReadU32(c.Slice(20), le);
        int count = ReadU16(c.Slice(24), le);
        int fragSize = ReadU16(c.Slice(26), le);
        uint sampleSize = ReadU32(c.Slice(28), le);
        if (startNum == 0 || count == 0 || fragSize == 0 || sampleSize == 0) return false;

        int at = 4 + octetsToInlineQos;
        if (at > c.Length) return false;
        var qos = (flags & 0x02) != 0 ? ParseInlineQos(c, ref at, le) : default;

        long offset = (long)(startNum - 1) * fragSize;
        if (offset >= sampleSize) return false;
        long expected = Math.Min((long)count * fragSize, sampleSize - offset);
        if (at + expected > c.Length) return false;

        frag = new RtpsDataFrag
        {
            ReaderId = readerId,
            WriterId = writerId,
            SequenceNumber = sn,
            Timestamp = timestamp,
            FragmentStartingNum = startNum,
            FragmentsInSubmessage = count,
            FragmentSize = fragSize,
            SampleSize = sampleSize,
            Fragments = c.Slice(at, (int)expected).ToArray(),
            KeyHash = qos.KeyHash,
            Disposed = qos.Disposed,
            RelatedGuid = qos.RelatedGuid,
            RelatedSn = qos.RelatedSn,
        };
        return true;
    }

    private struct InlineQos
    {
        public byte[]? KeyHash;
        public bool Disposed;
        public RtpsGuid? RelatedGuid;
        public long RelatedSn;
    }

    // Walks an inline-QoS parameter list to its sentinel, lifting the parameters we act on.
    private static InlineQos ParseInlineQos(ReadOnlySpan<byte> c, ref int at, bool le)
    {
        var q = new InlineQos();
        while (at + 4 <= c.Length)
        {
            ushort pid = ReadU16(c.Slice(at), le);
            ushort plen = ReadU16(c.Slice(at + 2), le);
            at += 4;
            if (pid == Pid.Sentinel) break;
            if (pid == Pid.KeyHash && plen >= 16 && at + 16 <= c.Length)
                q.KeyHash = c.Slice(at, 16).ToArray();
            else if (pid == Pid.StatusInfo && plen >= 4 && at + 4 <= c.Length)
                q.Disposed = (c[at + 3] & 0x03) != 0;
            else if (pid == Pid.RelatedSampleIdentity && plen >= 24 && at + 24 <= c.Length)
            {
                q.RelatedGuid = RtpsGuid.ReadFrom(c.Slice(at, 16));
                q.RelatedSn = ReadSn(c.Slice(at + 16), le);
            }
            at += plen;
        }
        return q;
    }

    private static long ReadSn(ReadOnlySpan<byte> s, bool le) =>
        ((long)ReadI32(s, le) << 32) | ReadU32(s.Slice(4), le);
    private static ushort ReadU16(ReadOnlySpan<byte> s, bool le) =>
        le ? BinaryPrimitives.ReadUInt16LittleEndian(s) : BinaryPrimitives.ReadUInt16BigEndian(s);
    private static uint ReadU32(ReadOnlySpan<byte> s, bool le) =>
        le ? BinaryPrimitives.ReadUInt32LittleEndian(s) : BinaryPrimitives.ReadUInt32BigEndian(s);
    private static int ReadI32(ReadOnlySpan<byte> s, bool le) =>
        le ? BinaryPrimitives.ReadInt32LittleEndian(s) : BinaryPrimitives.ReadInt32BigEndian(s);
}
