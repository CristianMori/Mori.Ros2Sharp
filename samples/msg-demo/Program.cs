using System.Net;
using Mori.Ros2Sharp;

// msg-demo — validates the .msg-driven code generation end to end. The message classes used
// below (Ros2Messages.*) are produced at build time by the bundled source generator from
// msgs\demo_msgs\msg\AllTypes.msg plus its embedded-set dependencies.
//
//   msg-demo                          offline checks
//   msg-demo live [domain] [seconds] [peerIp…]
//                                     publish a generated geometry_msgs/Twist on /cmd_vel
//                                     (verify with: ros2 topic echo /cmd_vel geometry_msgs/msg/Twist)

if (args.Length > 0 && args[0].Equals("live", StringComparison.OrdinalIgnoreCase))
    return await Live(args);

int failures = 0;
void Check(string name, bool ok)
{
    Console.WriteLine($"{(ok ? "ok  " : "FAIL")}  {name}");
    if (!ok) failures++;
}

// Generated constants and type names.
Check("type name constants",
    Ros2Messages.std_msgs.String.RosType == "std_msgs/msg/String" &&
    Ros2Messages.std_msgs.String.DdsType == "std_msgs::msg::dds_::String_" &&
    Ros2Messages.demo_msgs.AllTypes.DdsType == "demo_msgs::msg::dds_::AllTypes_");
Check("msg constants", Ros2Messages.demo_msgs.AllTypes.SMALL == 7 &&
    Ros2Messages.demo_msgs.AllTypes.GREETING == "hi there");

// Field defaults from the .msg file.
var fresh = new Ros2Messages.demo_msgs.AllTypes();
Check("field defaults", fresh.Flag && fresh.Label == "default label" && fresh.BoundedLabel == "");
Check("array initializers", fresh.Raw.Length == 4 && fresh.Triplet.Length == 3 && fresh.Blob.Length == 0);

// Byte equality: generated std_msgs/String vs hand-rolled CDR.
var hand = new CdrWriter(CdrEncapsulation.CdrLe);
hand.Write("chatter test");
byte[] generated = new Ros2Messages.std_msgs.String { Data = "chatter test" }.ToBytes();
Check("string bytes match hand-rolled CDR", generated.AsSpan().SequenceEqual(hand.ToArray()));

// Byte equality: generated geometry_msgs/Twist vs hand-rolled CDR (six aligned doubles).
var twist = new Ros2Messages.geometry_msgs.Twist();
twist.Linear.X = 0.25; twist.Linear.Y = -1.5; twist.Linear.Z = 3.0;
twist.Angular.X = 0.5; twist.Angular.Y = 0.75; twist.Angular.Z = -0.125;
var handTwist = new CdrWriter(CdrEncapsulation.CdrLe);
handTwist.Write(0.25); handTwist.Write(-1.5); handTwist.Write(3.0);
handTwist.Write(0.5); handTwist.Write(0.75); handTwist.Write(-0.125);
Check("twist bytes match hand-rolled CDR", twist.ToBytes().AsSpan().SequenceEqual(handTwist.ToArray()));

// Alignment: in AllTypes, "wide" (float64) follows "tiny" (uint8 at body offset 1, after the
// bool) and must land on the next 8-byte boundary — body offset 8, payload offset 12.
var aligned = new Ros2Messages.demo_msgs.AllTypes { Tiny = 0x11, Wide = 2.5 };
byte[] alignedBytes = aligned.ToBytes();
Check("float64 aligns to 8 after odd offset",
    alignedBytes[5] == 0x11 && BitConverter.ToDouble(alignedBytes, 12) == 2.5);

// Nested message with covariance: nav_msgs/Odometry round-trip (float64[36] arrays, strings,
// builtin_interfaces/Time inside the header).
var odom = new Ros2Messages.nav_msgs.Odometry();
odom.Header.Stamp.Sec = 1_700_000_000;
odom.Header.Stamp.Nanosec = 987_654_321;
odom.Header.FrameId = "odom";
odom.ChildFrameId = "base_link";
odom.Pose.Pose.Position.X = 1.5;
odom.Pose.Pose.Orientation.W = 1.0;
odom.Pose.Covariance[35] = 42.5;
odom.Twist.Twist.Angular.Z = -0.3;
var odomBack = Ros2Messages.nav_msgs.Odometry.FromBytes(odom.ToBytes());
Check("odometry round-trip",
    odomBack.Header.Stamp.Sec == 1_700_000_000 && odomBack.Header.Stamp.Nanosec == 987_654_321 &&
    odomBack.Header.FrameId == "odom" && odomBack.ChildFrameId == "base_link" &&
    odomBack.Pose.Pose.Position.X == 1.5 && odomBack.Pose.Pose.Orientation.W == 1.0 &&
    odomBack.Pose.Covariance[35] == 42.5 && odomBack.Twist.Twist.Angular.Z == -0.3);

// The whole kitchen sink through the wire and back.
var all = new Ros2Messages.demo_msgs.AllTypes
{
    Flag = false,
    Tiny = 9,
    Wide = -0.5,
    Label = "labeled",
    BoundedLabel = "bounded",
    Raw = new byte[] { 1, 2, 3, 4 },
    Blob = new byte[] { 5, 6, 7 },
    BoundedBlob = new byte[] { 8 },
    Triplet = new short[] { -1, 0, 1 },
    Samples = new[] { 1.5, 2.5 },
    Names = new[] { "a", "bc", "def" },
};
all.Note.Data = "nested string message";
all.Twist.Linear.X = 7.25;
all.Odom.ChildFrameId = "cf";
all.Timeout.Sec = 3;
all.Timeout.Nanosec = 500_000_000;
var allBack = Ros2Messages.demo_msgs.AllTypes.FromBytes(all.ToBytes());
Check("all-types round-trip",
    !allBack.Flag && allBack.Tiny == 9 && allBack.Wide == -0.5 &&
    allBack.Label == "labeled" && allBack.BoundedLabel == "bounded" &&
    allBack.Raw.AsSpan().SequenceEqual(all.Raw) && allBack.Blob.AsSpan().SequenceEqual(all.Blob) &&
    allBack.BoundedBlob.AsSpan().SequenceEqual(all.BoundedBlob) &&
    allBack.Triplet.AsSpan().SequenceEqual(all.Triplet) &&
    allBack.Samples.AsSpan().SequenceEqual(all.Samples) &&
    allBack.Names.AsSpan().SequenceEqual(all.Names) &&
    allBack.Note.Data == "nested string message" && allBack.Twist.Linear.X == 7.25 &&
    allBack.Odom.ChildFrameId == "cf" &&
    allBack.Timeout.Sec == 3 && allBack.Timeout.Nanosec == 500_000_000);

// Fixed-length enforcement.
bool threw = false;
try { new Ros2Messages.demo_msgs.AllTypes { Raw = new byte[3] }.ToBytes(); }
catch (InvalidOperationException) { threw = true; }
Check("fixed array length enforced", threw);

// A parse round through the runtime catalog (same parser the generators use).
var spec = Mori.Ros2Sharp.Msg.MsgSpec.Parse("demo_msgs/Probe",
    "int32 A=1\nstring<=5 s \"x\"\nuint8[<=9] b\nfloat64[2] f\ngeometry_msgs/Point p\n");
Check("parser: bounds, defaults, arrays",
    spec.Constants.Count == 1 && spec.Fields.Count == 4 &&
    spec.Fields[0].BaseType == "string" && spec.Fields[0].DefaultText == "\"x\"" &&
    spec.Fields[1].BaseType == "uint8" && spec.Fields[1].IsArray && spec.Fields[1].FixedLength == -1 &&
    spec.Fields[2].FixedLength == 2 &&
    spec.Fields[3].BaseType == "geometry_msgs/Point");

Console.WriteLine(failures == 0 ? "all checks passed" : $"{failures} check(s) FAILED");
return failures == 0 ? 0 : 1;

static async Task<int> Live(string[] args)
{
    int domain = args.Length > 1 ? int.Parse(args[1]) : 0;
    int seconds = args.Length > 2 ? int.Parse(args[2]) : 20;

    using var node = new Ros2Node("cs_msg_demo", "/", domain);
    foreach (string peer in args.Skip(3))
        node.AddPeer(IPAddress.Parse(peer));
    var pub = node.CreatePublisher("/cmd_vel", Ros2Messages.geometry_msgs.Twist.RosType);
    node.Start();
    Console.WriteLine($"msg-demo[live]: publishing generated Twist on /cmd_vel, domain {domain}, {seconds}s…");

    var until = DateTimeOffset.UtcNow.AddSeconds(seconds);
    int i = 0;
    while (DateTimeOffset.UtcNow < until)
    {
        var twist = new Ros2Messages.geometry_msgs.Twist();
        twist.Linear.X = 0.25 * ++i;
        twist.Angular.Z = -0.5;
        pub.Write(twist.ToBytes());
        await Task.Delay(500);
    }
    Console.WriteLine($"done: {i} sent, {pub.MatchedReaderCount} matched reader(s)");
    return 0;
}
