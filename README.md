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
| **Endpoints** | SEDP endpoint discovery in both directions, with reliability/durability compatibility matching |
| **Pub/sub** | Reliable and best-effort writers and readers: bounded history, HEARTBEAT/ACKNACK retransmission, GAP handling, duplicate suppression |
| **Graph** | `ros_discovery_info` participation — the node appears in `ros2 node list`, its topics in `ros2 topic list` |
| **Services** | Both sides: serve a service that `ros2 service call` can invoke, or call an existing ROS 2 service, with request/response correlation |
| **Codegen** | Typed message classes generated from `.msg` files — a bundled build-time source generator and the `ros2msggen` CLI, with the common interface packages embedded |

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
The same emitter is available as a CLI for offline generation:

```
ros2msggen -o Generated -n MyMessages path/to/my_package/msg
```

For DDS-level work (custom QoS combinations, non-ROS DDS systems, protocol experiments),
`RtpsParticipant` exposes the layer underneath: raw reader/writer endpoints, discovery events,
and locator control.

## The probe

`samples/dds-probe` exercises every layer against a live system:

```
dotnet run --project samples/dds-probe -- selftest      # offline wire-format checks
dotnet run --project samples/dds-probe -- 0 30          # discover participants on domain 0
dotnet run --project samples/dds-probe -- pub 0 30      # publish std_msgs/String on /chatter
dotnet run --project samples/dds-probe -- sub 0 30      # subscribe to /chatter and print
dotnet run --project samples/dds-probe -- node 0 30     # full node /cs_probe publishing /chatter
dotnet run --project samples/dds-probe -- serve 0 30    # serve /set_bool (std_srvs/srv/SetBool)
dotnet run --project samples/dds-probe -- call 0 30     # call /set_bool three times
```

Every mode accepts trailing peer addresses for networks where multicast cannot reach the
other side (containers, VMs, WSL): `dds-probe pub 0 30 192.168.1.20`. The library announces
itself to the well-known ports on those hosts directly, and discovery proceeds from there —
the same mechanism DDS initial-peer lists use.

## Scope and current limits

- **Transports**: UDPv4 unicast and multicast. Peers that also advertise shared-memory or
  TCP locators (Fast DDS does by default) interoperate fine — they fall back to UDP.
- **Middlewares**: RTPS is the standardized DDS wire protocol, so discovery and pub/sub are
  vendor-neutral by design; validation so far is against Fast DDS. Service correlation
  follows the convention of `rmw_fastrtps` (the ROS 2 default middleware). Non-DDS
  middlewares (e.g. Zenoh-based) are a different protocol family and out of scope.
- **QoS**: reliable / best-effort, volatile / transient-local, keep-last history. This covers
  the ROS 2 defaults and the common profiles; deadline, lifespan, and liveliness QoS are not
  implemented.
- **Message size**: no fragmentation support yet — payloads must fit one UDP datagram
  (roughly 60 KB).
- **Type descriptions**: endpoint matching is by topic and type name, which is how ROS 2
  Humble matches. The newer type-hash system is not implemented.
- **.msg grammar**: constants, defaults, bounded strings/arrays, and nested messages are
  supported; `wstring` fields and array default values are not.

## Building

.NET 8 SDK, no other prerequisites:

```
dotnet build
dotnet run --project samples/dds-probe -- selftest
```

## License

Apache-2.0 — © 2026 Cristian Mori
