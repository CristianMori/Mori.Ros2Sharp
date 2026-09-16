using System.Text;

namespace Mori.Ros2Sharp.Msg;

/// <summary>
/// A parsed .srv definition: the request and response message specs, split on the <c>---</c>
/// line. The halves of <c>pkg/Name</c> parse as <c>pkg/Name_Request</c> and
/// <c>pkg/Name_Response</c>, the names ROS 2 itself gives them (the installed share carries
/// <c>Name_Request.msg</c> next to each <c>Name.srv</c>).
/// </summary>
public sealed class SrvSpec
{
    public string FullType = "";
    public string Package = "";
    public string ShortName = "";
    public MsgSpec Request = null!;
    public MsgSpec Response = null!;

    public static SrvSpec Parse(string fullType, string text)
    {
        int slash = fullType.IndexOf('/');
        if (slash <= 0) throw new ArgumentException($"service type must be package/Name, got '{fullType}'");

        var request = new StringBuilder();
        var response = new StringBuilder();
        bool inResponse = false;
        foreach (string line in text.Replace("\r\n", "\n").Split('\n'))
        {
            // The separator is a line starting with --- (field lines always start with a type).
            if (!inResponse && line.TrimStart().StartsWith("---", StringComparison.Ordinal))
            {
                inResponse = true;
                continue;
            }
            (inResponse ? response : request).Append(line).Append('\n');
        }
        if (!inResponse) throw new FormatException($"{fullType}: no '---' separator in .srv text");

        return new SrvSpec
        {
            FullType = fullType,
            Package = fullType.Substring(0, slash),
            ShortName = fullType.Substring(slash + 1),
            Request = MsgSpec.Parse(fullType + "_Request", request.ToString()),
            Response = MsgSpec.Parse(fullType + "_Response", response.ToString()),
        };
    }
}
