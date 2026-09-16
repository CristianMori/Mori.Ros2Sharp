namespace Mori.Ros2Sharp;

/// <summary>
/// What every generated message class implements: wire-order serialization against the CDR
/// writer and reader. The typed node overloads (publish, subscribe, serve, call) accept any
/// implementation, generated or hand-written.
/// </summary>
public interface IRos2Message
{
    /// <summary>Writes the fields in wire order (the writer handles CDR alignment).</summary>
    void Serialize(CdrWriter w);

    /// <summary>Reads the fields in wire order.</summary>
    void Deserialize(CdrReader r);
}

/// <summary>Helpers for moving typed messages through the byte-level endpoints.</summary>
public static class Ros2MessageExtensions
{
    /// <summary>Serializes with the CDR_LE encapsulation header, ready for a writer.</summary>
    public static byte[] ToPayload(this IRos2Message message)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        message.Serialize(w);
        return w.ToArray();
    }

    /// <summary>Deserializes a payload that starts with its encapsulation header.</summary>
    public static T FromPayload<T>(byte[] payload) where T : IRos2Message, new()
    {
        var m = new T();
        m.Deserialize(new CdrReader(payload));
        return m;
    }

    /// <summary>Writes a typed message.</summary>
    public static long Write<T>(this RtpsWriterEndpoint writer, T message) where T : IRos2Message =>
        writer.Write(message.ToPayload());

    /// <summary>
    /// Subscribes a typed handler: each sample is deserialized into a fresh <typeparamref name="T"/>.
    /// Returns the underlying handler so it can be removed from <see cref="RtpsReaderEndpoint.DataReceived"/>.
    /// </summary>
    public static Action<RtpsGuid, byte[], RtpsTime?> OnMessage<T>(this RtpsReaderEndpoint reader, Action<T> handler)
        where T : IRos2Message, new()
    {
        Action<RtpsGuid, byte[], RtpsTime?> h = (_, payload, _) => handler(FromPayload<T>(payload));
        reader.DataReceived += h;
        return h;
    }
}
