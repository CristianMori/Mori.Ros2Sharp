using System.Buffers.Binary;
using System.Net;
using System.Security.Cryptography;

namespace Mori.Ros2Sharp;

/// <summary>An RTPS protocol version (we speak 2.1, the baseline every DDS accepts).</summary>
public readonly record struct ProtocolVersion(byte Major, byte Minor)
{
    public static readonly ProtocolVersion Local = new(2, 1);

    public override string ToString() => $"{Major}.{Minor}";
}

/// <summary>The DDS vendor id carried in every RTPS header (ours is unregistered).</summary>
public readonly record struct VendorId(byte Hi, byte Lo)
{
    public static readonly VendorId Local = new(0x4d, 0x52);

    public string Name => (Hi, Lo) switch
    {
        (0x01, 0x01) => "RTI Connext",
        (0x01, 0x02) => "OpenSplice",
        (0x01, 0x03) => "OpenDDS",
        (0x01, 0x0f) => "eProsima Fast DDS",
        (0x01, 0x10) => "Eclipse Cyclone DDS",
        (0x01, 0x11) => "GurumDDS",
        (0x01, 0x12) => "RustDDS",
        (0x4d, 0x52) => "Mori.Ros2Sharp",
        _ => $"vendor {Hi:x2}.{Lo:x2}",
    };

    public override string ToString() => Name;
}

/// <summary>The 12-byte prefix every entity within one participant shares.</summary>
public readonly struct GuidPrefix : IEquatable<GuidPrefix>
{
    private readonly ulong _hi;
    private readonly uint _lo;

    private GuidPrefix(ulong hi, uint lo) { _hi = hi; _lo = lo; }

    /// <summary>A fresh prefix: our vendor id in the first two bytes (per RTPS), then random.</summary>
    public static GuidPrefix NewUnique()
    {
        Span<byte> b = stackalloc byte[12];
        RandomNumberGenerator.Fill(b);
        b[0] = VendorId.Local.Hi;
        b[1] = VendorId.Local.Lo;
        return ReadFrom(b);
    }

    // Guid prefixes are opaque octet arrays on the wire — big-endian reads/writes here just
    // preserve byte order; they are never endian-swapped with the message.
    public static GuidPrefix ReadFrom(ReadOnlySpan<byte> s) =>
        new(BinaryPrimitives.ReadUInt64BigEndian(s), BinaryPrimitives.ReadUInt32BigEndian(s.Slice(8)));

    public void WriteTo(Span<byte> d)
    {
        BinaryPrimitives.WriteUInt64BigEndian(d, _hi);
        BinaryPrimitives.WriteUInt32BigEndian(d.Slice(8), _lo);
    }

    public bool Equals(GuidPrefix other) => _hi == other._hi && _lo == other._lo;
    public override bool Equals(object? obj) => obj is GuidPrefix g && Equals(g);
    public override int GetHashCode() => HashCode.Combine(_hi, _lo);
    public static bool operator ==(GuidPrefix a, GuidPrefix b) => a.Equals(b);
    public static bool operator !=(GuidPrefix a, GuidPrefix b) => !a.Equals(b);

    public override string ToString() => $"{_hi >> 32:x8}.{(uint)_hi:x8}.{_lo:x8}";
}

/// <summary>A 4-byte RTPS entity id (3-byte key + 1-byte kind), always big-endian on the wire.</summary>
public readonly record struct EntityId(uint Value)
{
    // The well-known builtin entity ids from the RTPS discovery module. The last byte encodes
    // the kind: c1 builtin participant, c2 builtin writer with key, c7 builtin reader with key;
    // user entities use 02/03 (writer keyed/keyless) and 04/07 (reader keyless/keyed).
    public static readonly EntityId Unknown = new(0);
    public static readonly EntityId Participant = new(0x000001c1);
    public static readonly EntityId SpdpWriter = new(0x000100c2);
    public static readonly EntityId SpdpReader = new(0x000100c7);
    public static readonly EntityId SedpPublicationsWriter = new(0x000003c2);
    public static readonly EntityId SedpPublicationsReader = new(0x000003c7);
    public static readonly EntityId SedpSubscriptionsWriter = new(0x000004c2);
    public static readonly EntityId SedpSubscriptionsReader = new(0x000004c7);
    public static readonly EntityId ParticipantMessageWriter = new(0x000200c2);
    public static readonly EntityId ParticipantMessageReader = new(0x000200c7);

    public void WriteTo(Span<byte> d) => BinaryPrimitives.WriteUInt32BigEndian(d, Value);
    public static EntityId ReadFrom(ReadOnlySpan<byte> s) => new(BinaryPrimitives.ReadUInt32BigEndian(s));

    public override string ToString() => Value.ToString("x8");
}

/// <summary>A full RTPS GUID: 12-byte participant prefix + 4-byte entity id.</summary>
public readonly record struct RtpsGuid(GuidPrefix Prefix, EntityId Entity)
{
    public void WriteTo(Span<byte> d)
    {
        Prefix.WriteTo(d);
        Entity.WriteTo(d.Slice(12));
    }

    public static RtpsGuid ReadFrom(ReadOnlySpan<byte> s) =>
        new(GuidPrefix.ReadFrom(s), EntityId.ReadFrom(s.Slice(12)));

    public override string ToString() => $"{Prefix}|{Entity}";
}

/// <summary>An RTPS timestamp: seconds + 2^-32-second fractions since the Unix epoch.</summary>
public readonly record struct RtpsTime(int Seconds, uint Fraction)
{
    /// <summary>Converts wall-clock time; fractions are NTP-style (1/2^32 of a second).</summary>
    public static RtpsTime FromDateTime(DateTimeOffset t)
    {
        long ticks = t.UtcTicks - DateTimeOffset.UnixEpoch.UtcTicks;
        long sub = ticks % TimeSpan.TicksPerSecond;
        return new RtpsTime((int)(ticks / TimeSpan.TicksPerSecond), (uint)(sub * (1L << 32) / TimeSpan.TicksPerSecond));
    }

    public DateTimeOffset ToDateTime() =>
        DateTimeOffset.UnixEpoch.AddTicks(Seconds * TimeSpan.TicksPerSecond
            + (long)(Fraction * (double)TimeSpan.TicksPerSecond / (1L << 32)));
}

/// <summary>An RTPS duration: seconds + 2^-32-second fractions (leases, QoS deadlines).</summary>
public readonly record struct RtpsDuration(int Seconds, uint Fraction)
{
    /// <summary>The RTPS "never expires" sentinel (seconds int32 max, fraction all ones).</summary>
    public static readonly RtpsDuration Infinite = new(0x7fffffff, 0xffffffff);

    public static RtpsDuration FromSeconds(double seconds)
    {
        int secs = (int)seconds;
        return new RtpsDuration(secs, (uint)((seconds - secs) * (1L << 32)));
    }

    public double ToSeconds() => Seconds + Fraction / (double)(1L << 32);
}

/// <summary>An RTPS locator: transport kind, port, and a 16-byte address (IPv4 in the last 4).</summary>
public sealed class Locator
{
    public const int KindUdpV4 = 1;
    public const int KindUdpV6 = 2;

    public int Kind { get; }
    public uint Port { get; }
    public IPAddress Address { get; }

    public Locator(int kind, uint port, IPAddress address)
    {
        Kind = kind;
        Port = port;
        Address = address;
    }

    public static Locator UdpV4(IPAddress address, int port) => new(KindUdpV4, (uint)port, address);

    /// <summary>The 24-byte wire form: kind int32, port uint32, 16-byte address.</summary>
    public void WriteTo(CdrWriter w)
    {
        w.Write(Kind);
        w.Write(Port);
        Span<byte> addr = stackalloc byte[16];
        byte[] raw = Address.GetAddressBytes();
        raw.CopyTo(Kind == KindUdpV4 ? addr.Slice(12) : addr);
        w.WriteBytes(addr);
    }

    public static Locator ReadFrom(CdrReader r)
    {
        int kind = r.ReadInt32();
        uint port = r.ReadUInt32();
        byte[] addr = r.ReadBytes(16);
        var ip = kind == KindUdpV4 ? new IPAddress(addr.AsSpan(12)) : new IPAddress(addr);
        return new Locator(kind, port, ip);
    }

    public override string ToString() => $"{Address}:{Port}";
}

/// <summary>The RTPS well-known port mapping: PB 7400, DG 250, PG 2, offsets d0..d3 = 0/10/1/11.</summary>
public static class RtpsPorts
{
    public static readonly IPAddress DiscoveryMulticastGroup = IPAddress.Parse("239.255.0.1");

    public static int MetatrafficMulticast(int domain) => 7400 + 250 * domain;
    public static int DefaultMulticast(int domain) => 7400 + 250 * domain + 1;
    public static int MetatrafficUnicast(int domain, int participant) => 7400 + 250 * domain + 10 + 2 * participant;
    public static int DefaultUnicast(int domain, int participant) => 7400 + 250 * domain + 11 + 2 * participant;
}

/// <summary>The builtin-endpoint bits a participant advertises in SPDP.</summary>
[Flags]
public enum BuiltinEndpoints : uint
{
    None = 0,
    ParticipantAnnouncer = 1u << 0,
    ParticipantDetector = 1u << 1,
    PublicationsAnnouncer = 1u << 2,
    PublicationsDetector = 1u << 3,
    SubscriptionsAnnouncer = 1u << 4,
    SubscriptionsDetector = 1u << 5,
    ParticipantMessageWriter = 1u << 10,
    ParticipantMessageReader = 1u << 11,
}
