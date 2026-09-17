using System.Diagnostics;
using System.Security.Cryptography;
using System.Text;
using Mori.Ros2Sharp;

namespace DdsProbe;

/// <summary>
/// Two nodes in one process exchanging samples too large for a single datagram: a reliable
/// topic carrying <c>count</c> samples of <c>size</c> bytes, then a service call whose
/// request and reply are both fragmented. Exercises DATA_FRAG on both ends over real sockets.
/// </summary>
internal static class Loopback
{
    /// <summary>
    /// A drop hook that discards the given percentage of datagrams carrying fragments (and
    /// nothing else, so discovery is undisturbed) and counts what it dropped.
    /// </summary>
    public sealed class FragmentLoss
    {
        private readonly Random _rng = new(12345);
        private readonly int _percent;
        public int Dropped;
        public int Seen;
        // Submessage tallies over everything that passed through the hook (dropped or not).
        public int Fragments, FragmentSubmessages, Data, Heartbeats, HeartbeatFrags, AckNacks, AckNackMissing, NackFrags, NackFragMissing, Gaps;

        public FragmentLoss(int percent) => _percent = percent;

        /// <summary>Per-sample fragment arrival record: distinct fragments, total incl. repeats, time span.</summary>
        public sealed class PerSample
        {
            public readonly HashSet<uint> Distinct = new();
            public int Total;
            public uint FragmentCount;
            public DateTimeOffset First, Last;
        }

        public readonly SortedDictionary<long, PerSample> Samples = new();
        public bool TrackSamples { get; set; }

        /// <summary>Every NACK_FRAG and requesting ACKNACK that passed through, with a timestamp.</summary>
        public readonly List<string> Requests = new();

        public bool Drop(byte[] datagram)
        {
            if (!RtpsMessage.TryParse(datagram, out var m)) return false;
            lock (this)
            {
                if (TrackSamples)
                    foreach (var f in m!.DataFrags)
                    {
                        if (!Samples.TryGetValue(f.SequenceNumber, out var ps))
                        {
                            ps = new PerSample { First = DateTimeOffset.UtcNow, FragmentCount = (f.SampleSize + (uint)f.FragmentSize - 1) / (uint)f.FragmentSize };
                            Samples[f.SequenceNumber] = ps;
                        }
                        for (int i = 0; i < f.FragmentsInSubmessage; i++) ps.Distinct.Add(f.FragmentStartingNum + (uint)i);
                        ps.Total += f.FragmentsInSubmessage;
                        ps.Last = DateTimeOffset.UtcNow;
                    }
                Fragments += m!.DataFrags.Sum(f => f.FragmentsInSubmessage);
                FragmentSubmessages += m.DataFrags.Count;
                if (Requests.Count < 400)
                {
                    string t = DateTimeOffset.UtcNow.ToString("HH:mm:ss.fff");
                    foreach (var nf in m.NackFrags)
                        Requests.Add($"{t} NACK_FRAG sn {nf.SequenceNumber}: {nf.Missing.Count} fragments from {(nf.Missing.Count > 0 ? nf.Missing[0] : 0)}");
                    foreach (var an in m.AckNacks.Where(a => a.Missing.Count > 0))
                        Requests.Add($"{t} ACKNACK base {an.BaseSn}: missing {string.Join(",", an.Missing.Take(8))}");
                    foreach (var hb in m.Heartbeats)
                        Requests.Add($"{t} HEARTBEAT wr {hb.WriterId} rd {hb.ReaderId} {hb.FirstSn}..{hb.LastSn}{(hb.Final ? " final" : "")}");
                    foreach (var d in m.Data)
                        Requests.Add($"{t} DATA wr {d.WriterId} rd {d.ReaderId} sn {d.SequenceNumber} ({d.Payload.Length} bytes)");
                    foreach (var g in m.Gaps)
                        Requests.Add($"{t} GAP wr {g.WriterId} rd {g.ReaderId} [{g.GapStart}, {g.GapListBase})");
                }
                Gaps += m.Gaps.Count;
                Data += m.Data.Count;
                Heartbeats += m.Heartbeats.Count;
                HeartbeatFrags += m.HeartbeatFrags.Count;
                AckNacks += m.AckNacks.Count;
                AckNackMissing += m.AckNacks.Sum(a => a.Missing.Count);
                NackFrags += m.NackFrags.Count;
                NackFragMissing += m.NackFrags.Sum(n => n.Missing.Count);
            }
            if (m.DataFrags.Count == 0) return false;
            Interlocked.Increment(ref Seen);
            if (_rng.Next(100) >= _percent) return false;
            Interlocked.Increment(ref Dropped);
            return true;
        }

        public string Summary =>
            $"fragments {Fragments} in {FragmentSubmessages} submessages, data {Data}, gap {Gaps}, heartbeat {Heartbeats}, heartbeat_frag {HeartbeatFrags}, " +
            $"acknack {AckNacks} (requesting {AckNackMissing} samples), nack_frag {NackFrags} (requesting {NackFragMissing} fragments)";
    }

    private static Action<string, bool>? _report;

    /// <summary>Runs the whole exchange; <paramref name="report"/> (name, passed) sees each check as it runs.</summary>
    public static async Task<int> Run(int size, int lossPercent, int count = 10, Action<string, bool>? report = null)
    {
        _report = report;
        using var talker = new Ros2Node("loop_talker");
        using var listener = new Ros2Node("loop_listener");
        // Unicast to the local slots as well: discovery then works where multicast does not
        // loop back (some CI runners).
        talker.AddPeer(System.Net.IPAddress.Loopback);
        listener.AddPeer(System.Net.IPAddress.Loopback);
        var loss = new FragmentLoss(lossPercent);
        var listenerOut = new FragmentLoss(0);
        talker.Participant.DropOutgoing = loss.Drop;
        listener.Participant.DropOutgoing = listenerOut.Drop;

        var pub = talker.CreatePublisher("/big", "std_msgs/msg/String");
        var sub = listener.CreateSubscription("/big", "std_msgs/msg/String");
        var received = new List<(long Sn, string Sha, int Bytes)>();
        var gotAll = new TaskCompletionSource();
        sub.SampleReceived += (_, d) =>
        {
            var r = new CdrReader(d.Payload);
            lock (received)
            {
                received.Add((d.SequenceNumber, Sha(r.ReadString()), d.Payload.Length));
                if (received.Count == count) gotAll.TrySetResult();
            }
        };

        // A service whose reply is larger than its request, both fragmented.
        var service = listener.CreateService("/pad", "dds_probe/srv/Pad", request =>
        {
            var r = new CdrReader(request);
            string text = r.ReadString();
            var w = new CdrWriter(CdrEncapsulation.CdrLe);
            w.Write(true);
            w.Write(Pattern(size * 3 / 2, 99) + "|" + Sha(text));
            return w.ToArray();
        });
        var client = talker.CreateClient("/pad", "dds_probe/srv/Pad");

        talker.Start();
        listener.Start();
        Console.WriteLine($"loopback: {count} samples of {size:N0} chars each, fragment size {talker.Participant.FragmentSize:N0}, " +
                          $"simulated fragment loss {lossPercent}%");

        var sw = Stopwatch.StartNew();
        for (int i = 0; i < 100 && (sub.MatchedWriterCount == 0 || pub.MatchedReaderCount == 0 || !client.ServerAvailable); i++)
            await Task.Delay(100);
        if (sub.MatchedWriterCount == 0 || pub.MatchedReaderCount == 0)
        {
            Console.WriteLine("FAIL  endpoints did not match");
            return 1;
        }
        Console.WriteLine($"    matched after {sw.ElapsedMilliseconds} ms");
        bool confirmed = await pub.WaitForReadersAsync(1, TimeSpan.FromSeconds(5));
        Console.WriteLine($"    reader confirmed the match after {sw.ElapsedMilliseconds} ms: {confirmed}");

        var sent = new List<string>();
        for (int i = 1; i <= count; i++)
        {
            string text = Pattern(size, i);
            sent.Add(Sha(text));
            var w = new CdrWriter(CdrEncapsulation.CdrLe);
            w.Write(text);
            pub.Write(w.ToArray());
        }
        await Task.WhenAny(gotAll.Task, Task.Delay(TimeSpan.FromSeconds(30)));

        int failures = 0;
        lock (received)
        {
            string lossNote = lossPercent > 0 ? $", {loss.Dropped} of {loss.Seen} fragment datagrams dropped" : "";
            Check("reader confirmed the match before the first write", confirmed, ref failures);
            Check($"all {count} samples delivered ({received.Count} in {sw.ElapsedMilliseconds} ms{lossNote})", received.Count == count, ref failures);
            Check("every sample byte-exact", received.Select(x => x.Sha).OrderBy(x => x).SequenceEqual(sent.OrderBy(x => x)), ref failures);
            Check("samples delivered in order", received.Select(x => x.Sha).SequenceEqual(sent), ref failures);
        }

        Check("service server matched", client.ServerAvailable, ref failures);
        string reqText = Pattern(size, 7);
        var rw = new CdrWriter(CdrEncapsulation.CdrLe);
        rw.Write(reqText);
        try
        {
            byte[] reply = await client.CallAsync(rw.ToArray(), TimeSpan.FromSeconds(10));
            var rr = new CdrReader(reply);
            bool ok = rr.ReadBool();
            string message = rr.ReadString();
            int bar = message.LastIndexOf('|');
            Check("fragmented request reached the server intact", ok && bar > 0 && message[(bar + 1)..] == Sha(reqText), ref failures);
            Check("fragmented reply intact", bar == size * 3 / 2 && Sha(message[..bar]) == Sha(Pattern(size * 3 / 2, 99)), ref failures);
        }
        catch (TaskCanceledException)
        {
            Check("service call completed", false, ref failures);
        }

        Console.WriteLine($"    talker sent:   {loss.Summary}; fragment datagrams {loss.Seen}, dropped {loss.Dropped}");
        Console.WriteLine($"    listener sent: {listenerOut.Summary}");
        // Latched topic: three messages written before any subscription exists. A
        // transient-local subscription created afterwards gets all three; a volatile one
        // still matches (the writer offers more than it asks) but gets only what follows.
        var listenerIn = new FragmentLoss(0);
        listener.Participant.DropIncoming = listenerIn.Drop;
        var latched = talker.CreatePublisher("/latched", "std_msgs/msg/String", transientLocal: true, historyDepth: 10);
        Console.WriteLine($"    latched: publisher entity {latched.Guid.Entity}");
        for (int i = 1; i <= 3; i++)
        {
            var w = new CdrWriter(CdrEncapsulation.CdrLe);
            w.Write($"latched {i}");
            latched.Write(w.ToArray());
        }
        await Task.Delay(300);
        var lateTl = listener.CreateSubscription("/latched", "std_msgs/msg/String", transientLocal: true);
        var lateVolatile = listener.CreateSubscription("/latched", "std_msgs/msg/String");
        Console.WriteLine($"    latched: at creation the late subscriptions had {lateTl.MatchedWriterCount}/{lateVolatile.MatchedWriterCount} matched writer(s); " +
                          $"entities tl {lateTl.Guid.Entity}, volatile {lateVolatile.Guid.Entity}");
        var tlGot = new List<string>();
        var volatileGot = new List<string>();
        lateTl.DataReceived += (_, p, _) => { lock (tlGot) tlGot.Add(new CdrReader(p).ReadString()); };
        lateVolatile.DataReceived += (_, p, _) => { lock (volatileGot) volatileGot.Add(new CdrReader(p).ReadString()); };
        for (int i = 0; i < 100 && (lateTl.MatchedWriterCount == 0 || lateVolatile.MatchedWriterCount == 0); i++)
            await Task.Delay(50);
        bool bothConfirmed = await latched.WaitForReadersAsync(2, TimeSpan.FromSeconds(5));
        Console.WriteLine($"    latched: publisher sees {latched.MatchedReaderCount} matched, {latched.ConfirmedReaderCount} confirmed reader(s)");
        await Task.Delay(300);
        var w4 = new CdrWriter(CdrEncapsulation.CdrLe);
        w4.Write("latched 4");
        latched.Write(w4.ToArray());
        await Task.Delay(500);
        lock (tlGot) lock (volatileGot)
        {
            Check("latched: both late subscriptions matched and confirmed", bothConfirmed, ref failures);
            Check($"latched: transient-local subscription received history + new ({string.Join("|", tlGot)})",
                tlGot.SequenceEqual(new[] { "latched 1", "latched 2", "latched 3", "latched 4" }), ref failures);
            Check($"latched: volatile subscription received only the new sample ({string.Join("|", volatileGot)})",
                volatileGot.SequenceEqual(new[] { "latched 4" }), ref failures);
        }
        if (Environment.GetEnvironmentVariable("DDS_PROBE_TRACE") != null)
        {
            string wr = latched.Guid.Entity.ToString();
            lock (listenerIn)
                foreach (var line in listenerIn.Requests.Where(l => l.Contains($"wr {wr}")).Take(60))
                    Console.WriteLine($"      listener in  {line}");
        }

        // A transient-local subscription must not match a volatile publisher (it asks for
        // more than is offered), while a volatile one matches either.
        var plain = talker.CreatePublisher("/plain", "std_msgs/msg/String");
        var wantsHistory = listener.CreateSubscription("/plain", "std_msgs/msg/String", transientLocal: true);
        var plainSub = listener.CreateSubscription("/plain", "std_msgs/msg/String");
        for (int i = 0; i < 100 && plainSub.MatchedWriterCount == 0; i++) await Task.Delay(50);
        await Task.Delay(300);
        Check("durability rule: volatile subscription matched the volatile publisher", plainSub.MatchedWriterCount == 1, ref failures);
        Check("durability rule: transient-local subscription did not match the volatile publisher",
            wantsHistory.MatchedWriterCount == 0 && plain.MatchedReaderCount == 1, ref failures);

        // Withdrawal: an endpoint removed on one side unmatches on the other through an SEDP
        // dispose; a participant that leaves takes all its endpoints with it.
        int talkerSubsLost = 0, listenerPubsLost = 0, listenerParticipantsLost = 0;
        talker.Participant.SubscriptionLost += e => { if (e.TopicName == "rt/going") Interlocked.Increment(ref talkerSubsLost); };
        listener.Participant.PublicationLost += e => { if (e.TopicName == "rt/going") Interlocked.Increment(ref listenerPubsLost); };
        listener.Participant.ParticipantLost += _ => Interlocked.Increment(ref listenerParticipantsLost);
        var goingPub = talker.CreatePublisher("/going", "std_msgs/msg/String");
        var goingSub = listener.CreateSubscription("/going", "std_msgs/msg/String");
        for (int i = 0; i < 100 && (goingPub.MatchedReaderCount == 0 || goingSub.MatchedWriterCount == 0); i++) await Task.Delay(50);
        Check("withdrawal: /going matched both ways", goingPub.MatchedReaderCount == 1 && goingSub.MatchedWriterCount == 1, ref failures);
        listener.RemoveSubscription(goingSub);
        for (int i = 0; i < 100 && goingPub.MatchedReaderCount > 0; i++) await Task.Delay(50);
        Check("withdrawal: removed subscription unmatched from the publisher", goingPub.MatchedReaderCount == 0 && talkerSubsLost == 1, ref failures);
        var goingSub2 = listener.CreateSubscription("/going", "std_msgs/msg/String");
        for (int i = 0; i < 100 && goingSub2.MatchedWriterCount == 0; i++) await Task.Delay(50);
        talker.RemovePublisher(goingPub);
        for (int i = 0; i < 100 && goingSub2.MatchedWriterCount > 0; i++) await Task.Delay(50);
        Check("withdrawal: removed publisher unmatched from the subscription", goingSub2.MatchedWriterCount == 0 && listenerPubsLost == 1, ref failures);

        var third = new Ros2Node("loop_third");
        third.AddPeer(System.Net.IPAddress.Loopback);
        var thirdPub = third.CreatePublisher("/going", "std_msgs/msg/String");
        third.Start();
        for (int i = 0; i < 200 && (goingSub2.MatchedWriterCount == 0 || thirdPub.MatchedReaderCount == 0); i++) await Task.Delay(50);
        Check($"withdrawal: third node's publisher matched (sub sees {goingSub2.MatchedWriterCount} writer(s), pub sees {thirdPub.MatchedReaderCount} reader(s))",
            goingSub2.MatchedWriterCount == 1 && thirdPub.MatchedReaderCount == 1, ref failures);
        third.Dispose();
        for (int i = 0; i < 100 && goingSub2.MatchedWriterCount > 0; i++) await Task.Delay(50);
        Check("withdrawal: leaving participant took its publisher with it",
            goingSub2.MatchedWriterCount == 0 && listenerPubsLost == 2 && listenerParticipantsLost == 1, ref failures);

        Console.WriteLine(failures == 0 ? "loopback passed" : $"{failures} loopback check(s) FAILED");
        return failures == 0 ? 0 : 1;
    }

    /// <summary>A deterministic text of <paramref name="length"/> chars, distinct per <paramref name="seed"/>.</summary>
    public static string Pattern(int length, int seed)
    {
        var sb = new StringBuilder(length);
        string head = $"#{seed}:";
        sb.Append(head);
        for (int i = head.Length; i < length; i++)
            sb.Append((char)('a' + (i * 7 + seed) % 26));
        return sb.ToString(0, length);
    }

    /// <summary>The first 12 hex digits of SHA-256 over the UTF-8 bytes (what an rclpy peer can compute too).</summary>
    public static string Sha(string text) =>
        Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(text)))[..12].ToLowerInvariant();

    private static void Check(string name, bool ok, ref int failures)
    {
        Console.WriteLine($"{(ok ? "ok  " : "FAIL")}  {name}");
        if (!ok) failures++;
        _report?.Invoke(name, ok);
    }
}
