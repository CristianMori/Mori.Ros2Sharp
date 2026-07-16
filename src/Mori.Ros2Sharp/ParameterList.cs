using System.Buffers.Binary;

namespace Mori.Ros2Sharp;

/// <summary>Well-known DDS discovery parameter ids (PL_CDR).</summary>
public static class Pid
{
    public const ushort Sentinel = 0x0001;
    public const ushort ParticipantLeaseDuration = 0x0002;
    public const ushort TopicName = 0x0005;
    public const ushort TypeName = 0x0007;
    public const ushort DomainId = 0x000f;
    public const ushort ProtocolVersion = 0x0015;
    public const ushort VendorId = 0x0016;
    public const ushort Reliability = 0x001a;
    public const ushort Durability = 0x001d;
    public const ushort UserData = 0x002c;
    public const ushort UnicastLocator = 0x002f;
    public const ushort MulticastLocator = 0x0030;
    public const ushort DefaultUnicastLocator = 0x0031;
    public const ushort MetatrafficUnicastLocator = 0x0032;
    public const ushort MetatrafficMulticastLocator = 0x0033;
    public const ushort History = 0x0040;
    public const ushort DefaultMulticastLocator = 0x0048;
    public const ushort ParticipantGuid = 0x0050;
    public const ushort BuiltinEndpointSet = 0x0058;
    public const ushort EndpointGuid = 0x005a;
    public const ushort EntityName = 0x0062;
    public const ushort KeyHash = 0x0070;
    public const ushort StatusInfo = 0x0071;

    /// <summary>Vendor-range pid Fast DDS uses to correlate service replies with requests.</summary>
    public const ushort RelatedSampleIdentity = 0x800f;
}

/// <summary>
/// Writes a PL_CDR_LE parameter list: (id, length, value) triples with values padded to 4 bytes,
/// closed by the sentinel. The payload format of all DDS discovery data.
/// </summary>
public sealed class ParameterListWriter
{
    private readonly CdrWriter _w = new(CdrEncapsulation.PlCdrLe);

    /// <summary>One raw parameter; the declared length is the value padded to a 4-byte multiple.</summary>
    public void Add(ushort pid, ReadOnlySpan<byte> value)
    {
        int padded = (value.Length + 3) & ~3;
        _w.Write(pid);
        _w.Write((ushort)padded);
        _w.WriteBytes(value);
        for (int i = value.Length; i < padded; i++) _w.Write((byte)0);
    }

    public void Add(ushort pid, uint value)
    {
        Span<byte> b = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32LittleEndian(b, value);
        Add(pid, b);
    }

    public void Add(ushort pid, string value)
    {
        var w = new CdrWriter();
        w.Write(value);
        Add(pid, w.AsSpan());
    }

    public void Add(ushort pid, ProtocolVersion v) => Add(pid, stackalloc byte[] { v.Major, v.Minor });

    public void Add(ushort pid, VendorId v) => Add(pid, stackalloc byte[] { v.Hi, v.Lo });

    public void Add(ushort pid, RtpsGuid guid)
    {
        Span<byte> b = stackalloc byte[16];
        guid.WriteTo(b);
        Add(pid, b);
    }

    public void Add(ushort pid, Locator locator)
    {
        var w = new CdrWriter();
        locator.WriteTo(w);
        Add(pid, w.AsSpan());
    }

    public void Add(ushort pid, RtpsDuration d)
    {
        Span<byte> b = stackalloc byte[8];
        BinaryPrimitives.WriteInt32LittleEndian(b, d.Seconds);
        BinaryPrimitives.WriteUInt32LittleEndian(b.Slice(4), d.Fraction);
        Add(pid, b);
    }

    /// <summary>A sequence&lt;octet&gt; value: uint32 count + raw bytes.</summary>
    public void AddOctetSequence(ushort pid, ReadOnlySpan<byte> bytes)
    {
        var w = new CdrWriter();
        w.WriteLength(bytes.Length);
        w.WriteBytes(bytes);
        Add(pid, w.AsSpan());
    }

    /// <summary>Closes the list with the sentinel and returns the complete payload.</summary>
    public byte[] Finish()
    {
        _w.Write(Pid.Sentinel);
        _w.Write((ushort)0);
        return _w.ToArray();
    }
}

/// <summary>Iterates the parameters of a PL_CDR_LE payload (starting at its encapsulation header).</summary>
public sealed class ParameterListReader
{
    private readonly byte[] _b;
    private readonly int _end;
    private int _at;

    public ParameterListReader(byte[] buffer, int offset, int length)
    {
        _b = buffer;
        _end = offset + length;
        _at = offset + 4;
    }

    /// <summary>Advances to the next parameter; false at the sentinel or a truncated list.</summary>
    public bool Next(out ushort pid, out ArraySegment<byte> value)
    {
        pid = 0;
        value = default;
        if (_at + 4 > _end) return false;
        pid = BinaryPrimitives.ReadUInt16LittleEndian(_b.AsSpan(_at));
        ushort len = BinaryPrimitives.ReadUInt16LittleEndian(_b.AsSpan(_at + 2));
        _at += 4;
        if (pid == Pid.Sentinel || _at + len > _end) return false;
        value = new ArraySegment<byte>(_b, _at, len);
        _at += len;
        return true;
    }
}
