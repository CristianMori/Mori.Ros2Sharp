using Mori.Ros2Sharp;

namespace MsgDemo;

/// <summary>
/// The offline checks for the generated code: byte equality against hand-rolled CDR,
/// alignment, round-trips through every grammar feature, the empty-struct rule, and a typed
/// service call between two in-process nodes. The message classes (Ros2Messages.*) are
/// produced at build time by the source generator from msgs\demo_msgs plus the embedded set.
/// </summary>
internal static class Checks
{
    /// <summary>Runs every check; <paramref name="report"/> (name, passed) sees each one as it runs.</summary>
    public static async Task<int> Run(Action<string, bool>? report = null)
    {
        int failures = 0;
        void Check(string name, bool ok)
        {
            Console.WriteLine($"{(ok ? "ok  " : "FAIL")}  {name}");
            if (!ok) failures++;
            report?.Invoke(name, ok);
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

        // Services: generated from .srv files (Pad from this project, std_srvs from the embedded set).
        Check("service type constants",
            Ros2Messages.std_srvs.SetBool.RosType == "std_srvs/srv/SetBool" &&
            Ros2Messages.std_srvs.SetBool.Request.DdsType == "std_srvs::srv::dds_::SetBool_Request_" &&
            Ros2Messages.std_srvs.SetBool.Response.DdsType == "std_srvs::srv::dds_::SetBool_Response_" &&
            Ros2Messages.demo_msgs.Pad.Request.MAX_REPEAT == 200);
        var handReq = new CdrWriter(CdrEncapsulation.CdrLe);
        handReq.Write(true);
        Check("SetBool request bytes match hand-rolled CDR",
            new Ros2Messages.std_srvs.SetBool.Request { Data = true }.ToBytes().AsSpan().SequenceEqual(handReq.ToArray()));
        var handResp = new CdrWriter(CdrEncapsulation.CdrLe);
        handResp.Write(false); handResp.Write("nope");
        Check("SetBool response bytes match hand-rolled CDR",
            new Ros2Messages.std_srvs.SetBool.Response { Success = false, Message = "nope" }.ToBytes().AsSpan().SequenceEqual(handResp.ToArray()));

        // An empty struct is one placeholder octet on the ROS 2 wire, never zero bytes.
        byte[] emptyReq = new Ros2Messages.std_srvs.Trigger.Request().ToBytes();
        byte[] emptyMsg = new Ros2Messages.std_msgs.Empty().ToBytes();
        Check("empty struct serializes as one placeholder byte",
            emptyReq.Length == 5 && emptyReq[4] == 0 && emptyMsg.Length == 5 &&
            Ros2Messages.std_srvs.Trigger.Request.FromBytes(emptyReq) != null);

        var padReq = new Ros2Messages.demo_msgs.Pad.Request { Text = "abc", Repeat = 3 };
        padReq.Origin.X = 1.5;
        var padReqBack = Ros2Messages.demo_msgs.Pad.Request.FromBytes(padReq.ToBytes());
        Check("Pad request round-trip (nested geometry_msgs/Point, default repeat=1)",
            padReqBack.Text == "abc" && padReqBack.Repeat == 3 && padReqBack.Origin.X == 1.5 &&
            new Ros2Messages.demo_msgs.Pad.Request().Repeat == 1);

        // Actions: the holder, its three messages, and the derived types rosidl composes.
        Check("action type constants",
            Ros2Messages.nav2_msgs.Wait.RosType == "nav2_msgs/action/Wait" &&
            Ros2Messages.nav2_msgs.Wait.Goal.DdsType == "nav2_msgs::action::dds_::Wait_Goal_" &&
            Ros2Messages.nav2_msgs.Wait.SendGoal.Request.DdsType == "nav2_msgs::action::dds_::Wait_SendGoal_Request_" &&
            Ros2Messages.nav2_msgs.Wait.GetResult.Response.DdsType == "nav2_msgs::action::dds_::Wait_GetResult_Response_" &&
            Ros2Messages.nav2_msgs.Wait.FeedbackMessage.DdsType == "nav2_msgs::action::dds_::Wait_FeedbackMessage_" &&
            Ros2Messages.action_msgs.GoalStatus.STATUS_SUCCEEDED == 4 &&
            Ros2Messages.action_msgs.CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID == 2);
        // SendGoal.Request is the 16-byte goal id followed by the goal: the framing the action
        // client writes by hand must equal the generated class byte for byte.
        var sendGoal = new Ros2Messages.nav2_msgs.Wait.SendGoal.Request();
        for (int i = 0; i < 16; i++) sendGoal.GoalId.Uuid[i] = (byte)(i + 1);
        sendGoal.Goal.Time.Sec = 3; sendGoal.Goal.Time.Nanosec = 500_000_000;
        var handSendGoal = new CdrWriter(CdrEncapsulation.CdrLe);
        handSendGoal.WriteBytes(Enumerable.Range(1, 16).Select(i => (byte)i).ToArray());
        handSendGoal.Write(3); handSendGoal.Write(500_000_000u);
        Check("SendGoal request bytes match the client's framing", sendGoal.ToBytes().AsSpan().SequenceEqual(handSendGoal.ToArray()));
        var fbMsg = new Ros2Messages.nav2_msgs.Wait.FeedbackMessage();
        fbMsg.GoalId.Uuid[0] = 9; fbMsg.Feedback.TimeLeft.Sec = 2;
        var fbBack = Ros2Messages.nav2_msgs.Wait.FeedbackMessage.FromBytes(fbMsg.ToBytes());
        Check("FeedbackMessage round-trip", fbBack.GoalId.Uuid[0] == 9 && fbBack.Feedback.TimeLeft.Sec == 2);
        var statuses = new Ros2Messages.action_msgs.GoalStatusArray();
        statuses.StatusList = new[] { new Ros2Messages.action_msgs.GoalStatus { Status = 2 } };
        Check("GoalStatusArray round-trip",
            Ros2Messages.action_msgs.GoalStatusArray.FromBytes(statuses.ToBytes()).StatusList[0].Status == 2);

        // A typed service call between two in-process nodes.
        {
            // A private domain: the check must never meet a ROS 2 system on the same network.
            using var server = new Ros2Node("msg_demo_server", "/", 200);
            using var caller = new Ros2Node("msg_demo_caller", "/", 200);
            server.AddPeer(System.Net.IPAddress.Loopback);
            caller.AddPeer(System.Net.IPAddress.Loopback);
            server.CreateService<Ros2Messages.demo_msgs.Pad.Request, Ros2Messages.demo_msgs.Pad.Response>(
                "/pad", Ros2Messages.demo_msgs.Pad.RosType, req =>
                {
                    var resp = new Ros2Messages.demo_msgs.Pad.Response { Ok = true, Digest = $"{req.Text}/{req.Origin.X}" };
                    resp.Padded.Data = string.Concat(Enumerable.Repeat(req.Text, req.Repeat));
                    return resp;
                });
            var client = caller.CreateClient<Ros2Messages.demo_msgs.Pad.Request, Ros2Messages.demo_msgs.Pad.Response>(
                "/pad", Ros2Messages.demo_msgs.Pad.RosType);
            server.Start();
            caller.Start();
            for (int i = 0; i < 100 && !client.ServerAvailable; i++) await Task.Delay(50);
            bool typedOk = false;
            try
            {
                var resp = await client.CallAsync(padReq, TimeSpan.FromSeconds(5));
                typedOk = resp.Ok && resp.Digest == "abc/1.5" && resp.Padded.Data == "abcabcabc";
            }
            catch (TaskCanceledException) { }
            Check("typed service call between two nodes", typedOk);
        }

        Console.WriteLine(failures == 0 ? "all checks passed" : $"{failures} check(s) FAILED");
        return failures == 0 ? 0 : 1;
    }
}
