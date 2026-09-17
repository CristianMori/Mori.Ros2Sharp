using System.Security.Cryptography;

namespace Mori.Ros2Sharp;

/// <summary>The goal states of action_msgs/GoalStatus.</summary>
public static class Ros2GoalStatus
{
    public const sbyte Unknown = 0;
    public const sbyte Accepted = 1;
    public const sbyte Executing = 2;
    public const sbyte Canceling = 3;
    public const sbyte Succeeded = 4;
    public const sbyte Canceled = 5;
    public const sbyte Aborted = 6;

    public static bool IsTerminal(sbyte status) => status >= Succeeded;
}

/// <summary>
/// A ROS 2 action client: the send_goal, get_result, and cancel_goal services plus the
/// feedback and status topics under <c>&lt;action&gt;/_action/</c>, with the goal-id and status
/// framing handled here so only the user-defined goal, result, and feedback types are needed.
/// </summary>
public sealed class Ros2ActionClient<TGoal, TResult, TFeedback>
    where TGoal : IRos2Message
    where TResult : IRos2Message, new()
    where TFeedback : IRos2Message, new()
{
    private readonly Ros2Client _sendGoal;
    private readonly Ros2Client _getResult;
    private readonly Ros2Client _cancel;
    private readonly RtpsReaderEndpoint _feedback;
    private readonly RtpsReaderEndpoint _status;
    private readonly Dictionary<string, Ros2GoalHandle<TResult, TFeedback>> _goals = new();

    public string ActionName { get; }

    /// <summary>True once all three services and both topics are matched with a server.</summary>
    public bool ServerAvailable =>
        _sendGoal.ServerAvailable && _getResult.ServerAvailable && _cancel.ServerAvailable &&
        _feedback.MatchedWriterCount > 0 && _status.MatchedWriterCount > 0;

    internal Ros2ActionClient(Ros2Node node, string actionName, string actionType)
    {
        // "pkg/action/Name" → the derived service and message type names rosidl generates.
        string[] parts = actionType.Split('/');
        if (parts.Length != 3 || parts[1] != "action")
            throw new ArgumentException($"action type must be package/action/Name, got '{actionType}'");
        string derived = $"{parts[0]}/action/{parts[2]}";
        ActionName = actionName;

        _sendGoal = node.CreateClient($"{actionName}/_action/send_goal", derived + "_SendGoal");
        _getResult = node.CreateClient($"{actionName}/_action/get_result", derived + "_GetResult");
        _cancel = node.CreateClient($"{actionName}/_action/cancel_goal", "action_msgs/srv/CancelGoal");
        _feedback = node.CreateSubscription($"{actionName}/_action/feedback", derived + "_FeedbackMessage");
        // The status topic is transient-local on every action server.
        _status = node.CreateSubscription($"{actionName}/_action/status", "action_msgs/msg/GoalStatusArray", transientLocal: true);

        _feedback.DataReceived += (_, payload, _) =>
        {
            var r = new CdrReader(payload);
            byte[] id = r.ReadBytes(16);
            Ros2GoalHandle<TResult, TFeedback>? handle;
            lock (_goals) _goals.TryGetValue(Key(id), out handle);
            if (handle == null) return;
            var feedback = new TFeedback();
            feedback.Deserialize(r);
            handle.OnFeedback(feedback);
        };
        _status.DataReceived += (_, payload, _) =>
        {
            var r = new CdrReader(payload);
            int n = r.ReadLength();
            for (int i = 0; i < n; i++)
            {
                byte[] id = r.ReadBytes(16);
                r.ReadInt32(); r.ReadUInt32(); // stamp
                sbyte status = r.ReadInt8();
                Ros2GoalHandle<TResult, TFeedback>? handle;
                lock (_goals) _goals.TryGetValue(Key(id), out handle);
                handle?.OnStatus(status);
            }
        };
    }

    /// <summary>Waits until a server is fully matched, or the timeout passes.</summary>
    public async Task<bool> WaitForServerAsync(TimeSpan timeout)
    {
        var deadline = DateTimeOffset.UtcNow + timeout;
        while (!ServerAvailable)
        {
            if (DateTimeOffset.UtcNow >= deadline) return false;
            await Task.Delay(50).ConfigureAwait(false);
        }
        return true;
    }

    /// <summary>
    /// Sends a goal and returns its handle once the server has answered. Check
    /// <see cref="Ros2GoalHandle{TResult, TFeedback}.Accepted"/>; a rejected goal has no result.
    /// </summary>
    public async Task<Ros2GoalHandle<TResult, TFeedback>> SendGoalAsync(TGoal goal, Action<TFeedback>? onFeedback = null, TimeSpan? timeout = null)
    {
        byte[] id = new byte[16];
        RandomNumberGenerator.Fill(id);
        id[6] = (byte)((id[6] & 0x0f) | 0x40); // RFC 4122 version 4
        id[8] = (byte)((id[8] & 0x3f) | 0x80);

        var handle = new Ros2GoalHandle<TResult, TFeedback>(id, onFeedback, GetResultAsync, CancelAsync);
        lock (_goals) _goals[Key(id)] = handle;

        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.WriteBytes(id);
        goal.Serialize(w);
        byte[] response = await _sendGoal.CallAsync(w.ToArray(), timeout).ConfigureAwait(false);
        var r = new CdrReader(response);
        handle.Accepted = r.ReadBool();
        handle.AcceptedStamp = new RtpsTime(r.ReadInt32(), r.ReadUInt32());
        if (!handle.Accepted) lock (_goals) _goals.Remove(Key(id));
        return handle;
    }

    internal async Task<(sbyte Status, TResult Result)> GetResultAsync(byte[] id, TimeSpan? timeout)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.WriteBytes(id);
        byte[] response = await _getResult.CallAsync(w.ToArray(), timeout).ConfigureAwait(false);
        var r = new CdrReader(response);
        sbyte status = r.ReadInt8();
        var result = new TResult();
        result.Deserialize(r);
        lock (_goals) _goals.Remove(Key(id));
        return (status, result);
    }

    internal async Task<sbyte> CancelAsync(byte[] id, TimeSpan? timeout)
    {
        var w = new CdrWriter(CdrEncapsulation.CdrLe);
        w.WriteBytes(id);   // goal_info.goal_id
        w.Write(0); w.Write(0u); // goal_info.stamp = 0: this goal regardless of time
        byte[] response = await _cancel.CallAsync(w.ToArray(), timeout).ConfigureAwait(false);
        return new CdrReader(response).ReadInt8(); // return_code; goals_canceling follows
    }

    private static string Key(byte[] id) => Convert.ToHexString(id);
}

/// <summary>One goal sent through a <see cref="Ros2ActionClient{TGoal, TResult, TFeedback}"/>.</summary>
public sealed class Ros2GoalHandle<TResult, TFeedback>
    where TResult : IRos2Message, new()
    where TFeedback : IRos2Message, new()
{
    // The client's result and cancel calls, bound at creation so the handle needs no goal type.
    private readonly Func<byte[], TimeSpan?, Task<(sbyte Status, TResult Result)>> _getResult;
    private readonly Func<byte[], TimeSpan?, Task<sbyte>> _cancel;
    private readonly Action<TFeedback>? _onFeedback;

    /// <summary>The 16-byte goal id (a version-4 UUID).</summary>
    public byte[] GoalId { get; }

    /// <summary>Set from the send_goal response.</summary>
    public bool Accepted { get; internal set; }
    public RtpsTime AcceptedStamp { get; internal set; }

    /// <summary>The latest state seen on the status topic (a <see cref="Ros2GoalStatus"/> value).</summary>
    public sbyte Status { get; private set; }

    /// <summary>Raised on every status change seen on the status topic.</summary>
    public event Action<sbyte>? StatusChanged;

    internal Ros2GoalHandle(byte[] goalId, Action<TFeedback>? onFeedback,
        Func<byte[], TimeSpan?, Task<(sbyte Status, TResult Result)>> getResult,
        Func<byte[], TimeSpan?, Task<sbyte>> cancel)
    {
        GoalId = goalId;
        _onFeedback = onFeedback;
        _getResult = getResult;
        _cancel = cancel;
    }

    internal void OnFeedback(TFeedback feedback) => _onFeedback?.Invoke(feedback);

    internal void OnStatus(sbyte status)
    {
        if (status == Status) return;
        Status = status;
        StatusChanged?.Invoke(status);
    }

    /// <summary>
    /// Awaits the goal's terminal state and returns its status and result. The server answers
    /// only once the goal ends, so give this the time the goal needs.
    /// </summary>
    public Task<(sbyte Status, TResult Result)> GetResultAsync(TimeSpan? timeout = null) => _getResult(GoalId, timeout);

    /// <summary>
    /// Asks the server to cancel this goal. Returns the action_msgs/CancelGoal return code
    /// (0 = accepted); the goal then ends with status Canceled, observable through
    /// <see cref="GetResultAsync"/>.
    /// </summary>
    public Task<sbyte> CancelAsync(TimeSpan? timeout = null) => _cancel(GoalId, timeout);
}
