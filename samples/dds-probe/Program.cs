using System.Net;
using System.Security.Cryptography;
using Mori.Ros2Sharp;

// dds-probe — RTPS / ROS 2 interop probe.
//   dds-probe selftest                          offline encode/decode round-trip checks
//   dds-probe loopback [sizeBytes]              two in-process nodes exchange large samples + a service call
//   dds-probe [domain] [seconds] [peerIp…]      discovery: announce + list every participant
//   dds-probe pub [domain] [seconds] [peerIp…] [size]  publish std_msgs/String on /chatter at 2 Hz
//                                               (size > 0: a payload of that many chars, fragmented)
//   dds-probe sub [domain] [seconds] [peerIp…]  subscribe to /chatter; print text or size + checksum
//   dds-probe node [domain] [seconds] [peerIp…] full ROS 2 node "/cs_probe" publishing /chatter
//   dds-probe serve [domain] [seconds] [peerIp…] serve /set_bool (std_srvs/srv/SetBool)
//   dds-probe call [domain] [seconds] [peerIp…]  call /set_bool three times

string mode = args.Length > 0 && !char.IsDigit(args[0], 0) ? args[0].ToLowerInvariant() : "discover";
if (mode == "selftest")
    return DdsProbe.SelfTest.Run();
if (mode == "loopback")
    return await DdsProbe.Loopback.Run(
        args.Length > 1 ? int.Parse(args[1]) : 1_000_000,
        args.Length > 2 ? int.Parse(args[2]) : 0);

int argBase = mode == "discover" ? 0 : 1;
int domain = args.Length > argBase ? int.Parse(args[argBase]) : 0;
int seconds = args.Length > argBase + 1 ? int.Parse(args[argBase + 1]) : 30;
// Trailing arguments: dotted ones are peer addresses, a bare number is the pub payload size,
// loss=N drops N% of fragment datagrams (outgoing for pub, incoming for sub).
var trailing = args.Skip(argBase + 2).ToList();
var peerArgs = trailing.Where(a => a.Contains('.')).ToList();
int payloadSize = trailing.Where(a => !a.Contains('.') && !a.Contains('=') && a != "latched").Select(int.Parse).FirstOrDefault();
int lossPercent = trailing.Where(a => a.StartsWith("loss=")).Select(a => int.Parse(a[5..])).FirstOrDefault();
int fragmentSize = trailing.Where(a => a.StartsWith("frag=")).Select(a => int.Parse(a[5..])).FirstOrDefault(); // frag=N sets the fragment size
bool latched = trailing.Contains("latched"); // pub: transient-local, one message written at once; sub: transient-local subscription
int dropAfter = trailing.Where(a => a.StartsWith("drop=")).Select(a => int.Parse(a[5..])).FirstOrDefault(); // withdraw our endpoint after N s, keep running

if (mode == "node")
{
    using var node = new Ros2Node("cs_probe", "/", domain);
    foreach (string peer in args.Skip(argBase + 2))
        node.AddPeer(IPAddress.Parse(peer));
    node.Participant.ParticipantDiscovered += (p, from) =>
        Console.WriteLine($"[+] {p.Guid.Prefix}  vendor={p.VendorId}  name={p.EntityName ?? "-"}");
    var chatter = node.CreatePublisher("/chatter", "std_msgs/msg/String");
    node.Start();
    Console.WriteLine($"dds-probe[node]: /cs_probe on domain {domain}, guid {node.Participant.Guid}, running {seconds}s…");
    var stop = DateTimeOffset.UtcNow.AddSeconds(seconds);
    int n = 0;
    while (DateTimeOffset.UtcNow < stop)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.Write($"hello from /cs_probe #{++n}");
        chatter.Write(w.ToArray());
        await Task.Delay(1000);
    }
    Console.WriteLine($"done: {n} sent, {chatter.MatchedReaderCount} matched reader(s)");
    return 0;
}

if (mode == "serve")
{
    using var node = new Ros2Node("cs_probe", "/", domain);
    foreach (string peer in args.Skip(argBase + 2))
        node.AddPeer(IPAddress.Parse(peer));
    Ros2Service? svc = null;
    svc = node.CreateService("/set_bool", "std_srvs/srv/SetBool", request =>
    {
        var r = new CdrReader(request);
        bool data = r.ReadBool();
        Console.WriteLine($"    request: data={data} (reply readers matched: {svc!.Replies.MatchedReaderCount})");
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.Write(true);
        w.Write($"C# saw {data}");
        return w.ToArray();
    });
    svc.Requests.SampleReceived += (writerGuid, d) =>
        Console.WriteLine($"    [req detail] writer={writerGuid} sn={d.SequenceNumber} " +
                          $"related={(d.RelatedGuid is { } rg ? $"{rg} sn={d.RelatedSn}" : "none")}");
    node.Start();
    Console.WriteLine($"dds-probe[serve]: /cs_probe serving /set_bool on domain {domain}, {seconds}s…");
    await Task.Delay(TimeSpan.FromSeconds(seconds));
    return 0;
}

if (mode == "call")
{
    using var node = new Ros2Node("cs_probe_client", "/", domain);
    foreach (string peer in args.Skip(argBase + 2))
        node.AddPeer(IPAddress.Parse(peer));
    var client = node.CreateClient("/set_bool", "std_srvs/srv/SetBool");
    node.Start();
    Console.WriteLine($"dds-probe[call]: calling /set_bool on domain {domain}…");
    for (int i = 0; i < 60 && !client.ServerAvailable; i++)
        await Task.Delay(250);
    if (!client.ServerAvailable) { Console.WriteLine("    no server matched"); return 1; }
    Console.WriteLine("    server matched");
    for (int i = 1; i <= 3; i++)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.Write(i % 2 == 1);
        try
        {
            byte[] response = await client.CallAsync(w.ToArray(), TimeSpan.FromSeconds(8));
            var r = new CdrReader(response);
            Console.WriteLine($"    call {i} (data={i % 2 == 1}) -> success={r.ReadBool()}, message=\"{r.ReadString()}\"");
        }
        catch (TaskCanceledException)
        {
            Console.WriteLine($"    call {i} timed out");
        }
    }
    return 0;
}

using var participant = new RtpsParticipant(domain, "mori_dds_probe");
foreach (string peer in peerArgs)
    participant.AddPeer(IPAddress.Parse(peer));

participant.ParticipantDiscovered += (p, from) =>
    Console.WriteLine($"[+] {p.Guid.Prefix}  vendor={p.VendorId}  name={p.EntityName ?? "-"}  (from {from})");
participant.ParticipantLost += p =>
    Console.WriteLine($"[-] {p.Guid.Prefix}  gone (dispose or lease expiry)  {DateTime.Now:HH:mm:ss.fff}");
participant.PublicationDiscovered += e =>
    Console.WriteLine($"[pub] {e.TopicName}  ({e.TypeName})  {(e.Reliable ? "reliable" : "best-effort")}");
participant.SubscriptionDiscovered += e =>
    Console.WriteLine($"[sub] {e.TopicName}  ({e.TypeName})  {(e.Reliable ? "reliable" : "best-effort")}");
participant.PublicationLost += e =>
    Console.WriteLine($"[-pub] {e.TopicName}  ({e.TypeName})  {DateTime.Now:HH:mm:ss.fff}");
participant.SubscriptionLost += e =>
    Console.WriteLine($"[-sub] {e.TopicName}  ({e.TypeName})  {DateTime.Now:HH:mm:ss.fff}");
participant.ReceiveError += ex =>
    Console.WriteLine($"[!] receive error: {ex.GetType().Name}: {ex.Message}\n{ex.StackTrace}");

if (fragmentSize > 0) participant.FragmentSize = fragmentSize;
var loss = new DdsProbe.Loopback.FragmentLoss(lossPercent);
var outgoing = new DdsProbe.Loopback.FragmentLoss(0);
// The hooks also tally submessages; with loss=0 they drop nothing.
if (mode == "pub") { participant.DropOutgoing = loss.Drop; participant.DropIncoming = outgoing.Drop; }
if (mode == "sub") { loss.TrackSamples = true; participant.DropIncoming = loss.Drop; participant.DropOutgoing = outgoing.Drop; }

participant.Start();
Console.WriteLine($"dds-probe[{mode}]: domain {domain}, guid {participant.Guid}, " +
                  $"slot {participant.ParticipantId}, fragment size {participant.FragmentSize:N0}, running {seconds}s…" +
                  (lossPercent > 0 ? $" (dropping {lossPercent}% of fragment datagrams)" : ""));

switch (mode)
{
    case "pub":
    {
        var writer = participant.CreateWriter(Ros2Names.Topic("/chatter"), Ros2Names.Type("std_msgs/msg/String"),
            transientLocal: latched, historyDepth: 10);
        var until = DateTimeOffset.UtcNow.AddSeconds(seconds);
        int i = 0;
        if (latched)
        {
            // Written before anyone subscribes: transient-local subscriptions get it later.
            var w0 = new CdrWriter(CdrEncapsulation.CdrLe);
            w0.Write("latched hello from C#");
            writer.Write(w0.ToArray());
            Console.WriteLine("    wrote the latched message; waiting for subscriptions…");
            while (DateTimeOffset.UtcNow < until)
            {
                await Task.Delay(1000);
                Console.WriteLine($"    {writer.MatchedReaderCount} matched, {writer.ConfirmedReaderCount} confirmed reader(s)");
            }
            break;
        }
        // Wait for the first reader to confirm the match, so even the first sample lands.
        var waitSw = System.Diagnostics.Stopwatch.StartNew();
        bool ready = await writer.WaitForReadersAsync(1, TimeSpan.FromSeconds(10));
        Console.WriteLine($"    first reader confirmed after {waitSw.ElapsedMilliseconds} ms: {ready}");
        while (DateTimeOffset.UtcNow < until)
        {
            if (dropAfter > 0 && i >= dropAfter * 2)
            {
                Console.WriteLine($"    withdrawing the writer at {DateTime.Now:HH:mm:ss.fff}; staying alive");
                writer.Dispose();
                await Task.Delay(until - DateTimeOffset.UtcNow);
                break;
            }
            var w = new CdrWriter(CdrEncapsulation.CdrLe);
            string text = payloadSize > 0
                ? DdsProbe.Loopback.Pattern(payloadSize, ++i)
                : $"hello from C# #{++i}";
            w.Write(text);
            long sn = writer.Write(w.ToArray());
            string detail = payloadSize > 0 ? $", {text.Length} chars, sha256 {DdsProbe.Loopback.Sha(text)}" : "";
            Console.WriteLine($"    sent #{i} (sn {sn}{detail}, {writer.MatchedReaderCount} matched reader(s))");
            await Task.Delay(500);
        }
        var drain = System.Diagnostics.Stopwatch.StartNew();
        bool acked = await writer.WaitForAcknowledgmentsAsync(TimeSpan.FromSeconds(5));
        Console.WriteLine($"    all samples acknowledged: {acked} (waited {drain.ElapsedMilliseconds} ms)");
        Console.WriteLine($"    fragment datagrams: {loss.Seen} sent, {loss.Dropped} dropped");
        Console.WriteLine($"    sent:     {loss.Summary}");
        Console.WriteLine($"    received: {outgoing.Summary}");
        break;
    }
    case "sub":
    {
        var reader = participant.CreateReader(Ros2Names.Topic("/chatter"), Ros2Names.Type("std_msgs/msg/String"),
            transientLocal: latched);
        reader.SampleReceived += (writerGuid, d) =>
        {
            var r = new CdrReader(d.Payload);
            string text = r.ReadString();
            if (text.Length <= 80)
                Console.WriteLine($"    got sn {d.SequenceNumber}: \"{text}\"  (from {writerGuid.Prefix})");
            else
                Console.WriteLine($"    got sn {d.SequenceNumber}: {text.Length} chars, {d.Payload.Length} payload bytes, " +
                                  $"sha256 {DdsProbe.Loopback.Sha(text)}  (from {writerGuid.Prefix})");
        };
        if (dropAfter > 0)
        {
            await Task.Delay(TimeSpan.FromSeconds(dropAfter));
            Console.WriteLine($"    withdrawing the reader at {DateTime.Now:HH:mm:ss.fff} ({reader.MatchedWriterCount} matched); staying alive");
            reader.Dispose();
            await Task.Delay(TimeSpan.FromSeconds(Math.Max(0, seconds - dropAfter)));
        }
        else
            await Task.Delay(TimeSpan.FromSeconds(seconds));
        Console.WriteLine($"    fragment datagrams: {loss.Seen} received, {loss.Dropped} dropped");
        Console.WriteLine($"    received: {loss.Summary}");
        Console.WriteLine($"    sent back: {outgoing.Summary}");
        lock (loss)
            foreach (var (sn, ps) in loss.Samples.Take(20))
                Console.WriteLine($"    sample {sn}: {ps.Distinct.Count}/{ps.FragmentCount} distinct fragments, {ps.Total} received incl. repeats, " +
                                  $"first {ps.First:HH:mm:ss.fff} over {(ps.Last - ps.First).TotalMilliseconds:F0} ms");
        if (Environment.GetEnvironmentVariable("DDS_PROBE_TRACE") != null)
        {
            lock (loss) foreach (var line in loss.Requests.Take(120)) Console.WriteLine($"      in  {line}");
            lock (outgoing) foreach (var line in outgoing.Requests.Take(120)) Console.WriteLine($"      out {line}");
        }
        Console.WriteLine($"done: {reader.MatchedWriterCount} matched writer(s)");
        break;
    }
    default:
        await Task.Delay(TimeSpan.FromSeconds(seconds));
        Console.WriteLine($"done: {participant.Participants.Count} participant(s) visible");
        break;
}
return 0;
