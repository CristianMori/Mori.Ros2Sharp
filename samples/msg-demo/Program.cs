using System.Net;
using Mori.Ros2Sharp;

// msg-demo — validates the .msg/.srv-driven code generation end to end. The classes used
// below (Ros2Messages.*) are produced at build time by the bundled source generator from
// msgs\demo_msgs plus its embedded-set dependencies. The offline checks live in Checks.cs,
// which the test project runs as well.
//
//   msg-demo                          offline checks
//   msg-demo live [domain] [seconds] [peerIp…]
//                                     publish a generated geometry_msgs/Twist on /cmd_vel
//                                     (verify with: ros2 topic echo /cmd_vel geometry_msgs/msg/Twist)
//   msg-demo serve [domain] [seconds] [peerIp…]
//                                     typed std_srvs servers: /set_bool (SetBool), /trigger (Trigger)
//   msg-demo call [domain] [seconds] [peerIp…]
//                                     typed clients calling /set_bool and /trigger
//   msg-demo action [domain] [seconds] [peerIp…]
//                                     action client for a nav2_msgs/action/Wait server on /wait:
//                                     one goal to completion with feedback, one canceled

if (args.Length > 0 && args[0].Equals("live", StringComparison.OrdinalIgnoreCase))
    return await Live(args);
if (args.Length > 0 && args[0].Equals("action", StringComparison.OrdinalIgnoreCase))
    return await ActionDemo(args);
if (args.Length > 0 && args[0].Equals("serve", StringComparison.OrdinalIgnoreCase))
    return await Serve(args);
if (args.Length > 0 && args[0].Equals("call", StringComparison.OrdinalIgnoreCase))
    return await Call(args);
return await MsgDemo.Checks.Run();

static async Task<int> Serve(string[] args)
{
    int domain = args.Length > 1 ? int.Parse(args[1]) : 0;
    int seconds = args.Length > 2 ? int.Parse(args[2]) : 30;
    using var node = new Ros2Node("cs_msg_demo", "/", domain);
    foreach (string peer in args.Skip(3))
        node.AddPeer(IPAddress.Parse(peer));
    node.CreateService<Ros2Messages.std_srvs.SetBool.Request, Ros2Messages.std_srvs.SetBool.Response>(
        "/set_bool", Ros2Messages.std_srvs.SetBool.RosType, req =>
        {
            Console.WriteLine($"    /set_bool request: data={req.Data}");
            return new Ros2Messages.std_srvs.SetBool.Response { Success = true, Message = $"C# saw {req.Data}" };
        });
    node.CreateService<Ros2Messages.std_srvs.Trigger.Request, Ros2Messages.std_srvs.Trigger.Response>(
        "/trigger", Ros2Messages.std_srvs.Trigger.RosType, _ =>
        {
            Console.WriteLine("    /trigger request");
            return new Ros2Messages.std_srvs.Trigger.Response { Success = true, Message = "triggered by C#" };
        });
    node.Start();
    Console.WriteLine($"msg-demo[serve]: typed /set_bool and /trigger on domain {domain}, {seconds}s…");
    await Task.Delay(TimeSpan.FromSeconds(seconds));
    return 0;
}

static async Task<int> Call(string[] args)
{
    int domain = args.Length > 1 ? int.Parse(args[1]) : 0;
    using var node = new Ros2Node("cs_msg_demo_client", "/", domain);
    foreach (string peer in args.Skip(3))
        node.AddPeer(IPAddress.Parse(peer));
    var setBool = node.CreateClient<Ros2Messages.std_srvs.SetBool.Request, Ros2Messages.std_srvs.SetBool.Response>(
        "/set_bool", Ros2Messages.std_srvs.SetBool.RosType);
    var trigger = node.CreateClient<Ros2Messages.std_srvs.Trigger.Request, Ros2Messages.std_srvs.Trigger.Response>(
        "/trigger", Ros2Messages.std_srvs.Trigger.RosType);
    node.Start();
    for (int i = 0; i < 60 && !(setBool.ServerAvailable && trigger.ServerAvailable); i++) await Task.Delay(250);
    Console.WriteLine($"    servers matched: set_bool={setBool.ServerAvailable} trigger={trigger.ServerAvailable}");
    int failures = 0;
    for (int i = 1; i <= 2; i++)
    {
        try
        {
            var r = await setBool.CallAsync(new Ros2Messages.std_srvs.SetBool.Request { Data = i % 2 == 1 }, TimeSpan.FromSeconds(8));
            Console.WriteLine($"    /set_bool({i % 2 == 1}) -> success={r.Success} message=\"{r.Message}\"");
        }
        catch (TaskCanceledException) { Console.WriteLine($"    /set_bool call {i} timed out"); failures++; }
        try
        {
            var r = await trigger.CallAsync(new Ros2Messages.std_srvs.Trigger.Request(), TimeSpan.FromSeconds(8));
            Console.WriteLine($"    /trigger -> success={r.Success} message=\"{r.Message}\"");
        }
        catch (TaskCanceledException) { Console.WriteLine($"    /trigger call {i} timed out"); failures++; }
    }
    return failures == 0 ? 0 : 1;
}

static async Task<int> ActionDemo(string[] args)
{
    int domain = args.Length > 1 ? int.Parse(args[1]) : 0;
    using var node = new Ros2Node("cs_msg_demo_action", "/", domain);
    foreach (string peer in args.Skip(3))
        node.AddPeer(IPAddress.Parse(peer));
    var client = node.CreateActionClient<Ros2Messages.nav2_msgs.Wait.Goal, Ros2Messages.nav2_msgs.Wait.Result, Ros2Messages.nav2_msgs.Wait.Feedback>(
        "/wait", Ros2Messages.nav2_msgs.Wait.RosType);
    node.Start();
    Console.WriteLine("msg-demo[action]: waiting for a /wait action server…");
    if (!await client.WaitForServerAsync(TimeSpan.FromSeconds(15))) { Console.WriteLine("    no server"); return 1; }
    Console.WriteLine("    server matched");
    int failures = 0;

    // Goal 1: a 2-second wait to completion, feedback along the way.
    var goal = new Ros2Messages.nav2_msgs.Wait.Goal();
    goal.Time.Sec = 2;
    int feedbacks = 0;
    var handle = await client.SendGoalAsync(goal, fb => { feedbacks++; Console.WriteLine($"    feedback: {fb.TimeLeft.Sec}.{fb.TimeLeft.Nanosec / 100_000_000}s left"); }, TimeSpan.FromSeconds(5));
    handle.StatusChanged += s => Console.WriteLine($"    status -> {s}");
    Console.WriteLine($"    goal 1 accepted: {handle.Accepted}");
    var (status, result) = await handle.GetResultAsync(TimeSpan.FromSeconds(15));
    Console.WriteLine($"    goal 1 result: status={status} elapsed={result.TotalElapsedTime.Sec}.{result.TotalElapsedTime.Nanosec / 100_000_000}s, {feedbacks} feedback(s)");
    if (!handle.Accepted || status != Ros2GoalStatus.Succeeded || feedbacks == 0 || result.TotalElapsedTime.Sec < 1) failures++;

    // Goal 2: a 10-second wait canceled after one second.
    goal.Time.Sec = 10;
    var handle2 = await client.SendGoalAsync(goal, timeout: TimeSpan.FromSeconds(5));
    Console.WriteLine($"    goal 2 accepted: {handle2.Accepted}");
    await Task.Delay(1000);
    sbyte cancelCode = await handle2.CancelAsync(TimeSpan.FromSeconds(5));
    var (status2, result2) = await handle2.GetResultAsync(TimeSpan.FromSeconds(15));
    Console.WriteLine($"    goal 2 cancel code={cancelCode}, result status={status2} elapsed={result2.TotalElapsedTime.Sec}.{result2.TotalElapsedTime.Nanosec / 100_000_000}s");
    if (!handle2.Accepted || cancelCode != 0 || status2 != Ros2GoalStatus.Canceled || result2.TotalElapsedTime.Sec > 2) failures++;

    Console.WriteLine(failures == 0 ? "action demo passed" : $"{failures} action check(s) FAILED");
    return failures == 0 ? 0 : 1;
}

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
        pub.Write(twist);
        await Task.Delay(500);
    }
    Console.WriteLine($"done: {i} sent, {pub.MatchedReaderCount} matched reader(s)");
    return 0;
}
