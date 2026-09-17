using Mori.Ros2Sharp.Msg;
using Xunit;

namespace Mori.Ros2Sharp.Tests;

/// <summary>Direct tests of the shared .msg/.srv parsers and the emitter's fixed rules.</summary>
public sealed class ParserTests
{
    [Fact]
    public void ServiceSplitsOnSeparatorAndNamesTheHalvesLikeRos2()
    {
        var srv = SrvSpec.Parse("std_srvs/SetBool", "bool data\n---\nbool success\nstring message\n");
        Assert.Equal("std_srvs/SetBool_Request", srv.Request.FullType);
        Assert.Equal("std_srvs/SetBool_Response", srv.Response.FullType);
        Assert.Single(srv.Request.Fields);
        Assert.Equal(2, srv.Response.Fields.Count);
    }

    [Fact]
    public void ServiceWithoutSeparatorIsRejected()
    {
        Assert.Throws<FormatException>(() => SrvSpec.Parse("x/Bad", "bool data\n"));
    }

    [Fact]
    public void ActionSplitsIntoThreePartsAndBringsSupportTypes()
    {
        var act = ActionSpec.Parse("demo/Count", "int32 target\n---\nint32 total\n---\nint32 current\n");
        Assert.Equal("demo/Count_Goal", act.Goal.FullType);
        Assert.Equal("demo/Count_Result", act.Result.FullType);
        Assert.Equal("demo/Count_Feedback", act.Feedback.FullType);
        Assert.Throws<FormatException>(() => ActionSpec.Parse("demo/Bad", "int32 a\n---\nint32 b\n"));

        var catalog = new MsgCatalog(customAction: t => t == "demo/Count" ? "int32 target\n---\nint32 total\n---\nint32 current\n" : null);
        var emitted = CSharpEmitter.EmitClosure(Array.Empty<string>(), Array.Empty<string>(), new[] { "demo/Count" }, catalog, "Ns", "");
        var keys = emitted.Select(e => e.Key).ToList();
        Assert.Contains("demo/action/Count", keys);
        Assert.Contains("unique_identifier_msgs/UUID", keys);
        Assert.Contains("action_msgs/GoalStatusArray", keys);
        Assert.Contains("action_msgs/srv/CancelGoal", keys);
        string code = emitted.Single(e => e.Key == "demo/action/Count").Value;
        Assert.Contains("public static class SendGoal", code);
        Assert.Contains("global::Ns.demo.Count.Goal Goal", code);
        Assert.Contains("\"demo::action::dds_::Count_GetResult_Response_\"", code);
    }

    [Fact]
    public void ConstantVersusStringDefaultWithEquals()
    {
        var spec = MsgSpec.Parse("t/M", "int32 LIMIT=5\nstring note \"a=b\"\n");
        Assert.Single(spec.Constants);
        Assert.Equal("5", spec.Constants[0].ValueText);
        Assert.Single(spec.Fields);
        Assert.Equal("\"a=b\"", spec.Fields[0].DefaultText);
    }

    [Fact]
    public void HeaderShorthandAndPackageResolution()
    {
        var spec = MsgSpec.Parse("my_pkg/M", "Header header\nPoint p\ngeometry_msgs/Vector3 v\n");
        Assert.Equal("std_msgs/Header", spec.Fields[0].BaseType);
        Assert.Equal("my_pkg/Point", spec.Fields[1].BaseType);
        Assert.Equal("geometry_msgs/Vector3", spec.Fields[2].BaseType);
    }

    [Fact]
    public void EmptyStructEmitsOnePlaceholderOctet()
    {
        var catalog = new MsgCatalog(t => t == "t/Nothing" ? "" : null);
        var emitted = CSharpEmitter.EmitClosure(new[] { "t/Nothing" }, catalog, "Ns", "");
        string code = Assert.Single(emitted).Value;
        Assert.Contains("w.Write((byte)0);", code);
        Assert.Contains("r.ReadUInt8();", code);
        Assert.Contains("global::Mori.Ros2Sharp.IRos2Message", code);
    }

    [Fact]
    public void EmbeddedServicesAreEmittedAsHoldersWithNestedHalves()
    {
        var catalog = new MsgCatalog();
        var emitted = CSharpEmitter.EmitClosure(Array.Empty<string>(), EmbeddedMessages.ServiceTypes, catalog, "Ns", "");
        Assert.Equal(EmbeddedMessages.ServiceTypes.Length, emitted.Count);
        var trigger = emitted.Single(e => e.Key == "std_srvs/srv/Trigger").Value;
        Assert.Contains("public static class Trigger", trigger);
        Assert.Contains("public sealed class Request", trigger);
        Assert.Contains("\"std_srvs::srv::dds_::Trigger_Response_\"", trigger);
    }

    [Fact]
    public void ClosureIncludesNestedDependenciesOnce()
    {
        var catalog = new MsgCatalog();
        var emitted = CSharpEmitter.EmitClosure(new[] { "nav_msgs/Odometry" }, catalog, "Ns", "");
        var keys = emitted.Select(e => e.Key).ToList();
        Assert.Contains("std_msgs/Header", keys);
        Assert.Contains("geometry_msgs/Pose", keys);
        Assert.Contains("builtin_interfaces/Time", keys);
        Assert.Equal(keys.Count, keys.Distinct().Count());
    }
}
