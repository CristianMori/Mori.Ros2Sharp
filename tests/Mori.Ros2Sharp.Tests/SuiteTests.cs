using Xunit;
using Xunit.Abstractions;

// The suites open UDP sockets on the well-known RTPS ports, so they must not overlap.
[assembly: CollectionBehavior(DisableTestParallelization = true)]

namespace Mori.Ros2Sharp.Tests;

/// <summary>
/// Runs the sample check suites under xunit. Each suite is one test; a failure names every
/// check that did not pass, so the sample output and the test output say the same thing.
/// </summary>
public sealed class SuiteTests
{
    private readonly ITestOutputHelper _output;

    public SuiteTests(ITestOutputHelper output) => _output = output;

    [Fact]
    public void WireFormatSelfTest()
    {
        var failed = new List<string>();
        int rc = DdsProbe.SelfTest.Run((name, ok) => Record(name, ok, failed));
        Assert.True(rc == 0 && failed.Count == 0, "failed checks: " + string.Join("; ", failed));
    }

    [Fact]
    public async Task GeneratedCodeChecks()
    {
        var failed = new List<string>();
        int rc = await MsgDemo.Checks.Run((name, ok) => Record(name, ok, failed));
        Assert.True(rc == 0 && failed.Count == 0, "failed checks: " + string.Join("; ", failed));
    }

    [Fact]
    public async Task LoopbackLossless()
    {
        var failed = new List<string>();
        int rc = await DdsProbe.Loopback.Run(size: 300_000, lossPercent: 0, report: (name, ok) => Record(name, ok, failed));
        Assert.True(rc == 0 && failed.Count == 0, "failed checks: " + string.Join("; ", failed));
    }

    [Fact]
    public async Task LoopbackWithFragmentLoss()
    {
        var failed = new List<string>();
        int rc = await DdsProbe.Loopback.Run(size: 300_000, lossPercent: 30, report: (name, ok) => Record(name, ok, failed));
        Assert.True(rc == 0 && failed.Count == 0, "failed checks: " + string.Join("; ", failed));
    }

    private void Record(string name, bool ok, List<string> failed)
    {
        _output.WriteLine($"{(ok ? "ok  " : "FAIL")}  {name}");
        if (!ok) failed.Add(name);
    }
}
