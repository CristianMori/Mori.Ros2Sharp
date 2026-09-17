using System.Text;

namespace Mori.Ros2Sharp.Msg;

/// <summary>
/// A parsed .action definition: goal, result, and feedback message specs, split on the two
/// <c>---</c> lines. The parts of <c>pkg/Name</c> parse as <c>pkg/Name_Goal</c>,
/// <c>pkg/Name_Result</c>, and <c>pkg/Name_Feedback</c>. The derived types an action
/// server exposes (<c>Name_SendGoal</c>, <c>Name_GetResult</c>, <c>Name_FeedbackMessage</c>)
/// are fixed compositions of these and are synthesized by the emitter.
/// </summary>
public sealed class ActionSpec
{
    public string FullType = "";
    public string Package = "";
    public string ShortName = "";
    public MsgSpec Goal = null!;
    public MsgSpec Result = null!;
    public MsgSpec Feedback = null!;

    public static ActionSpec Parse(string fullType, string text)
    {
        int slash = fullType.IndexOf('/');
        if (slash <= 0) throw new ArgumentException($"action type must be package/Name, got '{fullType}'");

        var parts = new List<StringBuilder> { new StringBuilder() };
        foreach (string line in text.Replace("\r\n", "\n").Split('\n'))
        {
            if (line.TrimStart().StartsWith("---", StringComparison.Ordinal))
            {
                parts.Add(new StringBuilder());
                continue;
            }
            parts[parts.Count - 1].Append(line).Append('\n');
        }
        if (parts.Count != 3) throw new FormatException($"{fullType}: an .action needs exactly two '---' separators, found {parts.Count - 1}");

        return new ActionSpec
        {
            FullType = fullType,
            Package = fullType.Substring(0, slash),
            ShortName = fullType.Substring(slash + 1),
            Goal = MsgSpec.Parse(fullType + "_Goal", parts[0].ToString()),
            Result = MsgSpec.Parse(fullType + "_Result", parts[1].ToString()),
            Feedback = MsgSpec.Parse(fullType + "_Feedback", parts[2].ToString()),
        };
    }
}
