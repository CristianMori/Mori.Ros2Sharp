using System.Net;
using Mori.Ros2Sharp;

// dds-probe — RTPS / ROS 2 interop probe.
//   dds-probe selftest                          offline encode/decode round-trip checks
//   dds-probe [domain] [seconds] [peerIp…]      discovery: announce + list every participant
//   dds-probe pub [domain] [seconds] [peerIp…]  publish std_msgs/String on /chatter at 2 Hz
//   dds-probe sub [domain] [seconds] [peerIp…]  subscribe to /chatter and print messages
//   dds-probe node [domain] [seconds] [peerIp…] full ROS 2 node "/cs_probe" publishing /chatter
//   dds-probe serve [domain] [seconds] [peerIp…] serve /set_bool (std_srvs/srv/SetBool)
//   dds-probe call [domain] [seconds] [peerIp…]  call /set_bool three times

string mode = args.Length > 0 && !char.IsDigit(args[0], 0) ? args[0].ToLowerInvariant() : "discover";
if (mode == "selftest")
    return DdsProbe.SelfTest.Run();

int argBase = mode == "discover" ? 0 : 1;
int domain = args.Length > argBase ? int.Parse(args[argBase]) : 0;
int seconds = args.Length > argBase + 1 ? int.Parse(args[argBase + 1]) : 30;

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
foreach (string peer in args.Skip(argBase + 2))
    participant.AddPeer(IPAddress.Parse(peer));

participant.ParticipantDiscovered += (p, from) =>
    Console.WriteLine($"[+] {p.Guid.Prefix}  vendor={p.VendorId}  name={p.EntityName ?? "-"}  (from {from})");
participant.ParticipantLost += p =>
    Console.WriteLine($"[-] {p.Guid.Prefix}  lease expired");
participant.PublicationDiscovered += e =>
    Console.WriteLine($"[pub] {e.TopicName}  ({e.TypeName})  {(e.Reliable ? "reliable" : "best-effort")}");
participant.SubscriptionDiscovered += e =>
    Console.WriteLine($"[sub] {e.TopicName}  ({e.TypeName})  {(e.Reliable ? "reliable" : "best-effort")}");

participant.Start();
Console.WriteLine($"dds-probe[{mode}]: domain {domain}, guid {participant.Guid}, " +
                  $"slot {participant.ParticipantId}, running {seconds}s…");

switch (mode)
{
    case "pub":
    {
        var writer = participant.CreateWriter(Ros2Names.Topic("/chatter"), Ros2Names.Type("std_msgs/msg/String"));
        var until = DateTimeOffset.UtcNow.AddSeconds(seconds);
        int i = 0;
        while (DateTimeOffset.UtcNow < until)
        {
            var w = new CdrWriter(CdrEncapsulation.CdrLe);
            w.Write($"hello from C# #{++i}");
            long sn = writer.Write(w.ToArray());
            Console.WriteLine($"    sent #{i} (sn {sn}, {writer.MatchedReaderCount} matched reader(s))");
            await Task.Delay(500);
        }
        break;
    }
    case "sub":
    {
        var reader = participant.CreateReader(Ros2Names.Topic("/chatter"), Ros2Names.Type("std_msgs/msg/String"));
        reader.DataReceived += (writerGuid, payload, _) =>
        {
            var r = new CdrReader(payload);
            Console.WriteLine($"    got: \"{r.ReadString()}\"  (from {writerGuid.Prefix})");
        };
        await Task.Delay(TimeSpan.FromSeconds(seconds));
        Console.WriteLine($"done: {reader.MatchedWriterCount} matched writer(s)");
        break;
    }
    default:
        await Task.Delay(TimeSpan.FromSeconds(seconds));
        Console.WriteLine($"done: {participant.Participants.Count} participant(s) visible");
        break;
}
return 0;
