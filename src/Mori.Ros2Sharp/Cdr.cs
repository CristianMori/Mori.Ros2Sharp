using System.Buffers.Binary;
using System.Text;

namespace Mori.Ros2Sharp;

/// <summary>The serialization scheme declared by the 4-byte header that opens every RTPS payload.</summary>
public enum CdrEncapsulation : ushort
{
    /// <summary>Plain CDR, little-endian — ROS 2 message bodies.</summary>
    CdrLe = 0x0001,

    /// <summary>Parameter-list CDR, little-endian — DDS discovery payloads.</summary>
    PlCdrLe = 0x0003,
}

/// <summary>
/// Writes OMG CDR (XCDR1) little-endian: every primitive is aligned to its own size, counted
/// from the byte after the encapsulation header. This is the wire format of every ROS 2 message.
/// </summary>
public sealed class CdrWriter
{
    private byte[] _b;
    private int _len;
    private readonly int _origin;

    /// <summary>A writer that opens with the given encapsulation header.</summary>
    public CdrWriter(CdrEncapsulation encapsulation, int capacity = 256)
    {
        _b = new byte[Math.Max(capacity, 16)];
        _b[1] = (byte)encapsulation; // the identifier is big-endian on the wire: { 0x00, scheme }
        _len = _origin = 4;
    }

    /// <summary>A headerless writer (nested values, e.g. inside a discovery parameter).</summary>
    public CdrWriter(int capacity = 64) => _b = new byte[Math.Max(capacity, 16)];

    public int Length => _len;

    private void Ensure(int more)
    {
        if (_len + more <= _b.Length) return;
        int size = _b.Length * 2;
        while (size < _len + more) size *= 2;
        Array.Resize(ref _b, size);
    }

    /// <summary>Pads with zeros so the next write lands on an n-byte boundary.</summary>
    public void Align(int n)
    {
        int pad = (n - (_len - _origin) % n) % n;
        Ensure(pad);
        for (int i = 0; i < pad; i++) _b[_len++] = 0;
    }

    // Primitives: little-endian, each aligned to its own size. Booleans are a single octet
    // (0 or 1); there is no dedicated wide-char or decimal type in the ROS 2 mapping.
    public void Write(bool v) => Write(v ? (byte)1 : (byte)0);
    public void Write(byte v) { Ensure(1); _b[_len++] = v; }
    public void Write(sbyte v) => Write(unchecked((byte)v));
    public void Write(short v) { Align(2); Ensure(2); BinaryPrimitives.WriteInt16LittleEndian(_b.AsSpan(_len), v); _len += 2; }
    public void Write(ushort v) { Align(2); Ensure(2); BinaryPrimitives.WriteUInt16LittleEndian(_b.AsSpan(_len), v); _len += 2; }
    public void Write(int v) { Align(4); Ensure(4); BinaryPrimitives.WriteInt32LittleEndian(_b.AsSpan(_len), v); _len += 4; }
    public void Write(uint v) { Align(4); Ensure(4); BinaryPrimitives.WriteUInt32LittleEndian(_b.AsSpan(_len), v); _len += 4; }
    public void Write(long v) { Align(8); Ensure(8); BinaryPrimitives.WriteInt64LittleEndian(_b.AsSpan(_len), v); _len += 8; }
    public void Write(ulong v) { Align(8); Ensure(8); BinaryPrimitives.WriteUInt64LittleEndian(_b.AsSpan(_len), v); _len += 8; }
    public void Write(float v) { Align(4); Ensure(4); BinaryPrimitives.WriteSingleLittleEndian(_b.AsSpan(_len), v); _len += 4; }
    public void Write(double v) { Align(8); Ensure(8); BinaryPrimitives.WriteDoubleLittleEndian(_b.AsSpan(_len), v); _len += 8; }

    /// <summary>A CDR string: uint32 length that counts a terminating NUL, then the bytes + NUL.</summary>
    public void Write(string v)
    {
        byte[] utf8 = Encoding.UTF8.GetBytes(v ?? "");
        Write((uint)(utf8.Length + 1)); // CDR strings count a terminating NUL
        Ensure(utf8.Length + 1);
        utf8.CopyTo(_b.AsSpan(_len));
        _len += utf8.Length;
        _b[_len++] = 0;
    }

    /// <summary>The uint32 element-count prefix of a sequence.</summary>
    public void WriteLength(int count) => Write((uint)count);

    /// <summary>Raw bytes, no alignment and no count prefix (fixed-size arrays, nested blobs).</summary>
    public void WriteBytes(ReadOnlySpan<byte> bytes)
    {
        Ensure(bytes.Length);
        bytes.CopyTo(_b.AsSpan(_len));
        _len += bytes.Length;
    }

    /// <summary>The bytes written so far, without copying. Invalidated by further writes.</summary>
    public ReadOnlySpan<byte> AsSpan() => _b.AsSpan(0, _len);

    /// <summary>A copy of the bytes written so far.</summary>
    public byte[] ToArray() => _b.AsSpan(0, _len).ToArray();
}

/// <summary>Reads OMG CDR (XCDR1) little-endian (the mirror of <see cref="CdrWriter"/>).</summary>
public sealed class CdrReader
{
    private readonly byte[] _b;
    private readonly int _end;
    private readonly int _origin;
    private int _at;

    /// <summary>Reads a payload that starts with the 4-byte encapsulation header.</summary>
    public CdrReader(byte[] payload) : this(payload, 0, payload.Length, hasEncapsulation: true) { }

    public CdrReader(byte[] buffer, int offset, int length, bool hasEncapsulation)
    {
        _b = buffer;
        _end = offset + length;
        if (hasEncapsulation)
        {
            EncapsulationId = (ushort)((buffer[offset] << 8) | buffer[offset + 1]);
            _at = _origin = offset + 4;
        }
        else
        {
            _at = _origin = offset;
        }
    }

    /// <summary>The encapsulation identifier (0x0001 CDR_LE, 0x0003 PL_CDR_LE, …); 0 when headerless.</summary>
    public ushort EncapsulationId { get; }

    /// <summary>Bytes not yet consumed (0 after a complete parse).</summary>
    public int Remaining => _end - _at;

    /// <summary>Skips padding so the next read starts on an n-byte boundary.</summary>
    public void Align(int n) => _at += (n - (_at - _origin) % n) % n;

    // Primitives mirror CdrWriter: little-endian, aligned to their own size.
    public bool ReadBool() => _b[_at++] != 0;
    public byte ReadUInt8() => _b[_at++];
    public sbyte ReadInt8() => unchecked((sbyte)_b[_at++]);
    public short ReadInt16() { Align(2); short v = BinaryPrimitives.ReadInt16LittleEndian(_b.AsSpan(_at)); _at += 2; return v; }
    public ushort ReadUInt16() { Align(2); ushort v = BinaryPrimitives.ReadUInt16LittleEndian(_b.AsSpan(_at)); _at += 2; return v; }
    public int ReadInt32() { Align(4); int v = BinaryPrimitives.ReadInt32LittleEndian(_b.AsSpan(_at)); _at += 4; return v; }
    public uint ReadUInt32() { Align(4); uint v = BinaryPrimitives.ReadUInt32LittleEndian(_b.AsSpan(_at)); _at += 4; return v; }
    public long ReadInt64() { Align(8); long v = BinaryPrimitives.ReadInt64LittleEndian(_b.AsSpan(_at)); _at += 8; return v; }
    public ulong ReadUInt64() { Align(8); ulong v = BinaryPrimitives.ReadUInt64LittleEndian(_b.AsSpan(_at)); _at += 8; return v; }
    public float ReadFloat32() { Align(4); float v = BinaryPrimitives.ReadSingleLittleEndian(_b.AsSpan(_at)); _at += 4; return v; }
    public double ReadFloat64() { Align(8); double v = BinaryPrimitives.ReadDoubleLittleEndian(_b.AsSpan(_at)); _at += 8; return v; }

    public string ReadString()
    {
        int len = (int)ReadUInt32();
        // The count includes a terminating NUL; tolerate writers that omit it.
        int chars = len > 0 && _b[_at + len - 1] == 0 ? len - 1 : len;
        string v = Encoding.UTF8.GetString(_b, _at, chars);
        _at += len;
        return v;
    }

    /// <summary>Raw bytes, no alignment (fixed-size arrays, nested blobs).</summary>
    public byte[] ReadBytes(int count)
    {
        byte[] v = _b.AsSpan(_at, count).ToArray();
        _at += count;
        return v;
    }

    /// <summary>The uint32 element-count prefix of a sequence.</summary>
    public int ReadLength() => (int)ReadUInt32();
}
