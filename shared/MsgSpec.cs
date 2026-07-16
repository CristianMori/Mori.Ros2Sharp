namespace Mori.Ros2Sharp.Msg;

/// <summary>The ROS 2 builtin field types (everything else is a nested message).</summary>
public static class MsgBuiltins
{
    private static readonly HashSet<string> All = new HashSet<string>
    {
        "bool", "int8", "uint8", "int16", "uint16", "int32", "uint32", "int64", "uint64",
        "float32", "float64", "string", "wstring",
        "byte", "char", // both are octets on the wire in ROS 2 (unlike ROS 1, where byte = int8)
    };

    public static bool Contains(string type) => All.Contains(type);
}

/// <summary>A constant declared in a .msg file (<c>int32 FOO=42</c>).</summary>
public sealed class MsgConstant
{
    public string Type = "";
    public string Name = "";
    /// <summary>The value exactly as written (string constants may keep surrounding quotes).</summary>
    public string ValueText = "";
}

/// <summary>One field of a .msg definition.</summary>
public sealed class MsgField
{
    /// <summary>The type exactly as written, brackets and bounds included
    /// (e.g. <c>float64[36]</c>, <c>string&lt;=256</c>, <c>uint8[&lt;=100]</c>).</summary>
    public string RawType = "";

    /// <summary>Element type without brackets or bounds, package-resolved for nested messages
    /// (e.g. <c>float64</c>, <c>string</c>, <c>geometry_msgs/Vector3</c>).</summary>
    public string BaseType = "";

    public string Name = "";
    public bool IsArray;

    /// <summary>-1 for variable-length (unbounded or bounded) arrays; the length for fixed ones.
    /// Bounded arrays serialize exactly like unbounded ones — the bound is a contract, not wire.</summary>
    public int FixedLength = -1;

    /// <summary>The default value text from the field line, when one was declared.</summary>
    public string? DefaultText;

    public bool IsBuiltin => MsgBuiltins.Contains(BaseType);
}

/// <summary>
/// A parsed ROS 2 .msg definition. The grammar is line-based: comments start at <c>#</c>, a line
/// whose first two tokens are followed by <c>=</c> is a constant, anything else is
/// <c>type name [default]</c>. Unqualified nested types resolve to the defining package, except
/// the <c>Header</c> shorthand which is always std_msgs/Header.
/// </summary>
public sealed class MsgSpec
{
    public string FullType = "";
    public string Package = "";
    public string ShortName = "";
    public readonly List<MsgConstant> Constants = new List<MsgConstant>();
    public readonly List<MsgField> Fields = new List<MsgField>();

    public static MsgSpec Parse(string fullType, string text)
    {
        int slash = fullType.IndexOf('/');
        if (slash <= 0) throw new ArgumentException($"message type must be package/Name, got '{fullType}'");
        var spec = new MsgSpec
        {
            FullType = fullType,
            Package = fullType.Substring(0, slash),
            ShortName = fullType.Substring(slash + 1),
        };

        foreach (string rawLine in text.Replace("\r\n", "\n").Split('\n'))
        {
            int hash = rawLine.IndexOf('#');
            string clean = (hash >= 0 ? rawLine.Substring(0, hash) : rawLine).Trim();
            if (clean.Length == 0) continue;

            // A constant iff exactly "TYPE NAME" stands before the '='. A '=' later in the line
            // can also be part of a field's string default — that is not a constant.
            int eq = clean.IndexOf('=');
            if (eq > 0 && SplitTokens(clean.Substring(0, eq)).Length == 2)
            {
                string[] lhs = SplitTokens(clean.Substring(0, eq));
                // String constants keep everything after '=' from the ORIGINAL line (a '#'
                // inside a string constant is part of the value, not a comment).
                string value = lhs[0].StartsWith("string", StringComparison.Ordinal) ||
                               lhs[0].StartsWith("wstring", StringComparison.Ordinal)
                    ? rawLine.Substring(rawLine.IndexOf('=') + 1).Trim()
                    : clean.Substring(eq + 1).Trim();
                spec.Constants.Add(new MsgConstant { Type = StripBound(lhs[0]), Name = lhs[1], ValueText = value });
                continue;
            }

            // Field: "type name" with an optional default value as the rest of the line.
            string[] tokens = clean.Split(new[] { ' ', '\t' }, 3, StringSplitOptions.RemoveEmptyEntries);
            if (tokens.Length < 2) throw new FormatException($"{fullType}: bad field line '{rawLine.Trim()}'");
            MsgField field = ParseField(tokens[0], tokens[1], spec.Package);
            if (tokens.Length == 3) field.DefaultText = tokens[2].Trim();
            spec.Fields.Add(field);
        }
        return spec;
    }

    private static MsgField ParseField(string rawType, string name, string package)
    {
        var f = new MsgField { RawType = rawType, Name = name };
        string baseType = rawType;

        int bracket = rawType.IndexOf('[');
        if (bracket >= 0)
        {
            f.IsArray = true;
            baseType = rawType.Substring(0, bracket);
            string inside = rawType.Substring(bracket + 1, rawType.Length - bracket - 2);
            // "[]" unbounded and "[<=N]" bounded both serialize with a count prefix.
            f.FixedLength = inside.Length == 0 || inside.StartsWith("<=", StringComparison.Ordinal)
                ? -1
                : int.Parse(inside, System.Globalization.CultureInfo.InvariantCulture);
        }

        baseType = StripBound(baseType); // "string<=256" carries the bound only as a contract

        f.BaseType = MsgBuiltins.Contains(baseType) ? baseType
            : baseType == "Header" ? "std_msgs/Header"
            : baseType.IndexOf('/') >= 0 ? baseType
            : package + "/" + baseType;
        return f;
    }

    /// <summary>Drops a <c>&lt;=N</c> bound suffix from a (w)string type token.</summary>
    private static string StripBound(string type)
    {
        int le = type.IndexOf("<=", StringComparison.Ordinal);
        return le >= 0 ? type.Substring(0, le) : type;
    }

    private static string[] SplitTokens(string s) =>
        s.Split(new[] { ' ', '\t' }, StringSplitOptions.RemoveEmptyEntries);
}

/// <summary>Supplies .msg text for a full type name; return null when unknown.</summary>
public delegate string? MsgTextResolver(string fullType);

/// <summary>
/// Resolves and caches message specs by full type name: a custom resolver first (files the
/// caller knows about), then the embedded common-message set. ROS 2 needs no md5 — endpoints
/// match by type name.
/// </summary>
public sealed class MsgCatalog
{
    private readonly MsgTextResolver? _custom;
    private readonly Dictionary<string, MsgSpec> _specs = new Dictionary<string, MsgSpec>();

    public MsgCatalog(MsgTextResolver? custom = null) => _custom = custom;

    public MsgSpec Get(string fullType)
    {
        if (_specs.TryGetValue(fullType, out MsgSpec? cached)) return cached;
        string? text = _custom?.Invoke(fullType) ?? EmbeddedMessages.Find(fullType);
        if (text is null)
            throw new KeyNotFoundException($"no .msg definition available for '{fullType}'");
        MsgSpec spec = MsgSpec.Parse(fullType, text);
        _specs[fullType] = spec;
        return spec;
    }
}
