# Mori.Ros2Sharp

A ROS 2 node in pure managed C# — the actual protocols, implemented directly on .NET sockets.
No bridge process, no native DDS libraries, no ROS installation required on the machine
running it.

A ROS 2 node is, on the wire, three things stacked together: the DDS-RTPS protocol over UDP,
OMG CDR serialization for message bodies, and a set of ROS conventions layered on top (topic
and type naming, graph advertisement, service correlation). This library implements each of
those layers itself, which is what makes the zero-dependency deployment possible: add the
package, `new Ros2Node(...)`, and the process shows up on the ROS graph like any other node.

## What works today

| Layer | Support |
|---|---|
| **Serialization** | OMG CDR (XCDR1) little-endian reader/writer with correct alignment — the wire format of every ROS 2 message |
| **Discovery** | SPDP participant discovery over multicast and unicast initial peers; lease tracking; immediate departure announcements (dispose) on shutdown |
| **Endpoints** | SEDP endpoint discovery in both directions, with reliability/durability compatibility matching; endpoint withdrawal (dispose) both ways, and unmatching when a participant leaves |
| **Pub/sub** | Reliable and best-effort writers and readers: bounded history, in-order delivery, HEARTBEAT/ACKNACK retransmission, GAP handling, duplicate suppression; latched topics (transient-local durability) in both directions; large samples (images, point clouds) fragmented and reassembled with DATA_FRAG/NACK_FRAG recovery |
| **Graph** | `ros_discovery_info` participation — the node appears in `ros2 node list`, its topics in `ros2 topic list` |
| **Services** | Both sides: serve a service that `ros2 service call` can invoke, or call an existing ROS 2 service, with request/response correlation |
| **Actions** | Action client: send goals, receive feedback and status, await results, cancel — against any ROS 2 action server |
| **Codegen** | Typed message, service, and action classes generated from `.msg`, `.srv`, and `.action` files — a bundled build-time source generator and the `ros2msggen` CLI, with the common interface packages embedded |

Interoperability is validated live against unmodified ROS 2 Humble nodes (Fast DDS, the
default middleware): `ros2 topic echo` prints what this library publishes, subscriptions
receive what `ros2 topic pub` sends, and services round-trip in both directions.

## Quick start

A minimal talker:

```csharp
using Mori.Ros2Sharp;

using var node = new Ros2Node("talker");
var chatter = node.CreatePublisher("/chatter", "std_msgs/msg/String");
node.Start();

for (int i = 0; ; i++)
{
    var msg = new CdrWriter(CdrEncapsulation.CdrLe);
    msg.Write($"hello {i}");
    chatter.Write(msg.ToArray());
    await Task.Delay(1000);
}
```

A listener:

```csharp
using var node = new Ros2Node("listener");
var sub = node.CreateSubscription("/chatter", "std_msgs/msg/String");
sub.DataReceived += (writer, payload, timestamp) =>
{
    var msg = new CdrReader(payload);
    Console.WriteLine(msg.ReadString());
};
node.Start();
```

Discovery is symmetric but not simultaneous, so the first message of a stream can be
written before the other side has finished matching, and a volatile subscription then never
sees it (the familiar ROS 2 "first message lost"). Two ways to handle it, as in any ROS 2
client library:

```csharp
// Wait until at least one subscription has confirmed the match, then publish.
await chatter.WaitForReadersAsync(1, TimeSpan.FromSeconds(5));
chatter.Write(first);

// Or make the topic latched: late subscriptions receive the last messages (here 1).
var tfStatic = node.CreatePublisher("/tf_static", "tf2_msgs/msg/TFMessage",
    transientLocal: true, historyDepth: 1);
var sub = node.CreateSubscription("/tf_static", "tf2_msgs/msg/TFMessage", transientLocal: true);
```

Messages that arrive before a handler is attached to `DataReceived` are held (the most recent
64) and delivered to the first handler, so latched history is never missed by subscribing on
one line and attaching on the next.

Publishers and subscriptions can be withdrawn while the node keeps running
(`node.RemovePublisher(pub)`, `node.RemoveSubscription(sub)`): the other side is told through
discovery and unmatches at once, exactly as it does when a whole node exits.

A service server (`std_srvs/srv/SetBool`):

```csharp
using var node = new Ros2Node("toggler");
node.CreateService("/set_bool", "std_srvs/srv/SetBool", request =>
{
    bool data = new CdrReader(request).ReadBool();
    var response = new CdrWriter(CdrEncapsulation.CdrLe);
    response.Write(true);                    // success
    response.Write($"received {data}");     // message
    return response.ToArray();
});
node.Start();
```

And the matching client:

```csharp
var client = node.CreateClient("/set_bool", "std_srvs/srv/SetBool");
var request = new CdrWriter(CdrEncapsulation.CdrLe);
request.Write(true);
byte[] response = await client.CallAsync(request.ToArray(), TimeSpan.FromSeconds(5));
```

Message bodies can be hand-serialized with `CdrWriter`/`CdrReader` as above — fields in
declaration order, primitives aligned automatically — or generated. Add `.msg` files to your
project as `AdditionalFiles` and the bundled source generator turns them into typed classes at
build time, dependencies included (the common interface packages — `builtin_interfaces`,
`std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `tf2_msgs` — are embedded, so
referencing `std_msgs/Header` just works):

```xml
<ItemGroup>
  <AdditionalFiles Include="msgs/**/*.msg" />
</ItemGroup>
```

```csharp
var twist = new Ros2Messages.geometry_msgs.Twist();
twist.Linear.X = 0.25;
pub.Write(twist.ToBytes());   // CDR, encapsulation included

var back = Ros2Messages.geometry_msgs.Twist.FromBytes(payload);
```

Generated classes carry `RosType`/`DdsType` constants, apply `.msg` field defaults, enforce
fixed array lengths, and round-trip byte-identically with every DDS implementation tested.

`.srv` files generate a holder class with nested `Request` and `Response` messages, and the
node has typed service overloads. The `std_srvs` services (`Empty`, `SetBool`, `Trigger`) are
always generated:

```csharp
using Ros2Messages.std_srvs;

node.CreateService<SetBool.Request, SetBool.Response>("/set_bool", SetBool.RosType,
    req => new SetBool.Response { Success = true, Message = $"received {req.Data}" });

var client = node.CreateClient<Trigger.Request, Trigger.Response>("/trigger", Trigger.RosType);
Trigger.Response reply = await client.CallAsync(new Trigger.Request(), TimeSpan.FromSeconds(5));
```

Typed publish and subscribe come as extension methods on the endpoints:
`pub.Write(twist)` and `sub.OnMessage<Twist>(t => …)`.

`.action` files generate the goal, result, and feedback classes plus the derived send-goal,
get-result, and feedback-message types, and the node provides an action client:

```csharp
using Ros2Messages.nav2_msgs;

var client = node.CreateActionClient<Wait.Goal, Wait.Result, Wait.Feedback>("/wait", Wait.RosType);
await client.WaitForServerAsync(TimeSpan.FromSeconds(10));

var goal = new Wait.Goal();
goal.Time.Sec = 2;
var handle = await client.SendGoalAsync(goal, fb => Console.WriteLine($"{fb.TimeLeft.Sec}s left"));
var (status, result) = await handle.GetResultAsync(TimeSpan.FromSeconds(30));
// status == Ros2GoalStatus.Succeeded; handle.CancelAsync() ends a running goal
```

The same emitter is available as a CLI for offline generation:

```
ros2msggen -o Generated -n MyMessages path/to/my_package
```

For DDS-level work (custom QoS combinations, non-ROS DDS systems, protocol experiments),
`RtpsParticipant` exposes the layer underneath: raw reader/writer endpoints, discovery events,
and locator control.

## The probe

`samples/msg-demo` checks the generated code offline (`msg-demo`) and live: `msg-demo live`
publishes a generated Twist, `msg-demo serve` / `msg-demo call` run typed `std_srvs` servers
and clients, `msg-demo action` drives a `nav2_msgs/action/Wait` server. `samples/dds-probe`
exercises every layer against a live system:

```
dotnet run --project samples/dds-probe -- selftest      # offline wire-format checks
dotnet run --project samples/dds-probe -- 0 30          # discover participants on domain 0
dotnet run --project samples/dds-probe -- pub 0 30      # publish std_msgs/String on /chatter
dotnet run --project samples/dds-probe -- sub 0 30      # subscribe to /chatter and print
dotnet run --project samples/dds-probe -- node 0 30     # full node /cs_probe publishing /chatter
dotnet run --project samples/dds-probe -- serve 0 30    # serve /set_bool (std_srvs/srv/SetBool)
dotnet run --project samples/dds-probe -- call 0 30     # call /set_bool three times
dotnet run --project samples/dds-probe -- loopback      # two in-process nodes exchange 1 MB samples
```

`pub` takes an optional payload size (`pub 0 30 <peer> 300000`) to publish fragmented
samples; `sub` prints the size and checksum of every sample it receives. `loss=N` on `pub`,
`sub`, or as the loopback's second argument (`loopback 1000000 30`) drops N% of fragment
datagrams to exercise retransmission. `latched` makes `pub` a transient-local publisher that
writes once and waits, and `sub` a transient-local subscription. `drop=N` withdraws the
endpoint after N seconds while the process stays alive, to watch the other side unmatch.

Every mode accepts trailing peer addresses for networks where multicast cannot reach the
other side (containers, VMs, WSL): `dds-probe pub 0 30 192.168.1.20`. The library announces
itself to the well-known ports on those hosts directly, and discovery proceeds from there —
the same mechanism DDS initial-peer lists use.

## Scope and current limits

- **Transports**: UDPv4 unicast and multicast. Peers that also advertise shared-memory or
  TCP locators (Fast DDS does by default) interoperate fine — they fall back to UDP. On hosts
  with many adapters (VPNs, hypervisors, containers) the library advertises at most four
  addresses, the ones routing to configured peers first, because Fast DDS only keeps the
  first four it sees; `RtpsParticipant.AdvertisedAddresses` pins the list explicitly.
- **Middlewares**: RTPS is the standardized DDS wire protocol, so discovery and pub/sub are
  vendor-neutral by design. Validated against Fast DDS (the ROS 2 default, including
  services) and Cyclone DDS (discovery, pub/sub, fragmented samples in both directions).
  Cyclone runs without well-known unicast ports by default and is found through multicast
  only, which the library sends on every interface. Service correlation follows the
  convention of `rmw_fastrtps`. Non-DDS middlewares (e.g. Zenoh-based) are a different
  protocol family and out of scope.
- **QoS**: reliable / best-effort, volatile / transient-local, keep-last history, with the
  DDS request/offer compatibility rule at matching. This covers the ROS 2 defaults and the
  common profiles; deadline, lifespan, and liveliness QoS are not implemented.
- **Message size**: samples larger than one datagram are fragmented (default fragment size
  64 KB, matching Fast DDS; Cyclone DDS receives them fine too) and reassembled on receipt;
  `RtpsParticipant.FragmentSize` lowers it for links where IP fragmentation is undesirable.
  Lost fragments are re-requested individually (NACK_FRAG); a reliable writer can
  `WaitForAcknowledgmentsAsync` before shutting down so retransmissions in progress complete.
  Cyclone's default writer configuration hands out a large sample in request-driven chunks,
  so its throughput toward any reader is bounded by its own history watermark and retransmit
  queue settings.
- **Type descriptions**: endpoint matching is by topic and type name, which is how ROS 2
  Humble matches. The newer type-hash system is not implemented.
- **Interface grammar**: constants, defaults, bounded strings/arrays, nested messages, and
  the `.srv`/`.action` splits are supported; `wstring` fields and array default values are
  not. Actions have a client; an action server is not implemented.

## Building and testing

.NET 8 SDK, no other prerequisites:

```
dotnet build
dotnet test tests/Mori.Ros2Sharp.Tests
```

The tests run the wire-format self-test, the generated-code checks, and the in-process
loopback (two nodes exchanging fragmented samples, with and without injected loss) under
xunit; the same suites are runnable by hand through `dds-probe selftest`, `msg-demo`, and
`dds-probe loopback`. The in-process nodes use ROS domain 200, so the tests neither see nor
disturb a ROS 2 system on the same network. CI runs them on every push.

## License

Apache-2.0 — © 2026 Cristian Mori
