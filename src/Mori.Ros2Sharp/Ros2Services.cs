namespace Mori.Ros2Sharp;

/// <summary>
/// A served ROS 2 service: requests arrive on the rq/ topic, the handler produces the response
/// payload, and the reply goes out on the rr/ topic carrying the request's sample identity so
/// the caller can correlate it.
/// </summary>
public sealed class Ros2Service
{
    public string ServiceName { get; }
    public RtpsReaderEndpoint Requests { get; }
    public RtpsWriterEndpoint Replies { get; }

    internal Ros2Service(string serviceName, RtpsReaderEndpoint requests, RtpsWriterEndpoint replies,
        Func<byte[], byte[]> handler)
    {
        ServiceName = serviceName;
        Requests = requests;
        Replies = replies;
        requests.SampleReceived += (writerGuid, d) =>
        {
            byte[] response = handler(d.Payload);
            // The reply's related identity: the guid comes from the request's inline QoS when set
            // (rmw_fastrtps puts the client's reply-reader guid there, with an UNKNOWN sequence
            // number), but the sequence number is the request's own unless a real one was sent.
            var identity = d.RelatedGuid ?? writerGuid;
            long sn = d.RelatedGuid != null && d.RelatedSn > 0 ? d.RelatedSn : d.SequenceNumber;
            replies.Write(response, identity, sn);
        };
    }
}

/// <summary>A ROS 2 service client: correlated request/response over the rq/rr topic pair.</summary>
public sealed class Ros2Client
{
    private readonly RtpsWriterEndpoint _requests;
    private readonly RtpsReaderEndpoint _replies;
    private readonly Dictionary<long, TaskCompletionSource<byte[]>> _pending = new();

    public string ServiceName { get; }

    /// <summary>True once a server's request reader and reply writer are both matched.</summary>
    public bool ServerAvailable => _requests.MatchedReaderCount > 0 && _replies.MatchedWriterCount > 0;

    internal Ros2Client(string serviceName, RtpsWriterEndpoint requests, RtpsReaderEndpoint replies)
    {
        ServiceName = serviceName;
        _requests = requests;
        _replies = replies;
        replies.SampleReceived += (_, d) =>
        {
            if (d.RelatedGuid != _requests.Guid) return; // someone else's reply
            TaskCompletionSource<byte[]>? tcs;
            lock (_pending)
                if (!_pending.Remove(d.RelatedSn, out tcs))
                    return;
            tcs!.TrySetResult(d.Payload);
        };
    }

    /// <summary>Sends a serialized request and awaits the correlated response payload.</summary>
    public async Task<byte[]> CallAsync(byte[] requestPayload, TimeSpan? timeout = null)
    {
        var tcs = new TaskCompletionSource<byte[]>(TaskCreationOptions.RunContinuationsAsynchronously);
        long sn;
        lock (_pending) // registration under the same lock the reply handler takes: no lost race
        {
            sn = _requests.WriteSelfRelated(requestPayload);
            _pending[sn] = tcs;
        }
        using var cts = new CancellationTokenSource(timeout ?? TimeSpan.FromSeconds(10));
        await using (cts.Token.Register(() =>
        {
            lock (_pending) _pending.Remove(sn);
            tcs.TrySetCanceled();
        }))
        {
            return await tcs.Task.ConfigureAwait(false);
        }
    }
}
