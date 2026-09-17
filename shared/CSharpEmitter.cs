using System.Text;

namespace Mori.Ros2Sharp.Msg;

/// <summary>
/// Emits a typed C# class per message: fields in declaration order, <c>RosType</c>/<c>DdsType</c>
/// constants, and Serialize/Deserialize against Mori.Ros2Sharp's CdrWriter/CdrReader (which own
/// the CDR alignment rules). Shared by the ros2msggen CLI tool and the Roslyn source generator so
/// both produce identical code. Messages land in <c>&lt;rootNamespace&gt;.&lt;package&gt;</c>;
/// the emitted set is closed over nested message dependencies automatically.
/// </summary>
public static class CSharpEmitter
{
    /// <summary>Emits <paramref name="rootTypes"/> plus every message they reference.
    /// Returns (fullType, code) pairs, deterministically ordered.</summary>
    public static List<KeyValuePair<string, string>> EmitClosure(
        IEnumerable<string> rootTypes, MsgCatalog catalog, string rootNamespace, string header) =>
        EmitClosure(rootTypes, Array.Empty<string>(), catalog, rootNamespace, header);

    /// <summary>Emits messages and services plus every message they reference.</summary>
    public static List<KeyValuePair<string, string>> EmitClosure(
        IEnumerable<string> messageTypes, IEnumerable<string> serviceTypes,
        MsgCatalog catalog, string rootNamespace, string header) =>
        EmitClosure(messageTypes, serviceTypes, Array.Empty<string>(), catalog, rootNamespace, header);

    /// <summary>
    /// Emits the given messages, services, and actions plus every message they reference.
    /// Keys are <c>package/Name</c> for messages, <c>package/srv/Name</c> for services, and
    /// <c>package/action/Name</c> for actions, so they never collide as file names. Any
    /// action brings the action_msgs support types with it.
    /// </summary>
    public static List<KeyValuePair<string, string>> EmitClosure(
        IEnumerable<string> messageTypes, IEnumerable<string> serviceTypes, IEnumerable<string> actionTypes,
        MsgCatalog catalog, string rootNamespace, string header)
    {
        var pending = new Stack<string>();
        var seen = new HashSet<string>();
        foreach (string t in messageTypes)
            if (seen.Add(t)) pending.Push(t);

        var services = new List<string>();
        void AddService(string s)
        {
            if (services.Contains(s)) return;
            services.Add(s);
            SrvSpec srv = catalog.GetService(s);
            foreach (MsgField f in srv.Request.Fields.Concat(srv.Response.Fields))
                if (!f.IsBuiltin && seen.Add(f.BaseType))
                    pending.Push(f.BaseType);
        }
        foreach (string s in serviceTypes) AddService(s);

        var actions = new List<string>();
        foreach (string a in actionTypes)
        {
            if (actions.Contains(a)) continue;
            actions.Add(a);
            ActionSpec act = catalog.GetAction(a);
            foreach (MsgField f in act.Goal.Fields.Concat(act.Result.Fields).Concat(act.Feedback.Fields))
                if (!f.IsBuiltin && seen.Add(f.BaseType))
                    pending.Push(f.BaseType);
        }
        if (actions.Count > 0)
        {
            foreach (string t in new[] { "unique_identifier_msgs/UUID", "builtin_interfaces/Time" }.Concat(EmbeddedMessages.ActionSupportMessages))
                if (seen.Add(t)) pending.Push(t);
            foreach (string s in EmbeddedMessages.ActionSupportServices) AddService(s);
        }

        var closure = new List<string>();
        while (pending.Count > 0)
        {
            string type = pending.Pop();
            closure.Add(type);
            foreach (MsgField f in catalog.Get(type).Fields)
                if (!f.IsBuiltin && seen.Add(f.BaseType))
                    pending.Push(f.BaseType);
        }
        closure.Sort(StringComparer.Ordinal);
        services.Sort(StringComparer.Ordinal);
        actions.Sort(StringComparer.Ordinal);

        var output = new List<KeyValuePair<string, string>>();
        foreach (string type in closure)
            output.Add(new KeyValuePair<string, string>(type, EmitMessageFile(catalog.Get(type), rootNamespace, header)));
        foreach (string s in services)
        {
            int slash = s.IndexOf('/');
            string key = s.Substring(0, slash) + "/srv/" + s.Substring(slash + 1);
            output.Add(new KeyValuePair<string, string>(key, EmitServiceFile(catalog.GetService(s), rootNamespace, header)));
        }
        foreach (string a in actions)
        {
            int slash = a.IndexOf('/');
            string key = a.Substring(0, slash) + "/action/" + a.Substring(slash + 1);
            output.Add(new KeyValuePair<string, string>(key, EmitActionFile(catalog.GetAction(a), rootNamespace, header)));
        }
        return output;
    }

    // An action is a static holder with Goal/Result/Feedback and the three derived types an
    // action server exposes, synthesized here exactly as rosidl composes them: SendGoal
    // (request: goal id + goal; response: accepted + stamp), GetResult (request: goal id;
    // response: status + result), and FeedbackMessage (goal id + feedback).
    private static string EmitActionFile(ActionSpec act, string rootNamespace, string header)
    {
        var sb = new StringBuilder();
        FileHeader(sb, rootNamespace, act.Package, header);
        string name = Identifier(act.ShortName);
        string holder = $"global::{rootNamespace}.{Identifier(act.Package)}.{name}";
        string ros = $"{act.Package}/action/{act.ShortName}";
        string dds = $"{act.Package}::action::dds_::{act.ShortName}";

        MsgField Ref(string baseType, string fieldName, string? csType = null) =>
            new MsgField { RawType = baseType, BaseType = baseType, Name = fieldName, CsType = csType };
        MsgSpec Synth(string suffix, params MsgField[] fields)
        {
            var s = new MsgSpec { FullType = $"{act.Package}/{act.ShortName}_{suffix}", Package = act.Package, ShortName = $"{act.ShortName}_{suffix}" };
            s.Fields.AddRange(fields);
            return s;
        }
        MsgField GoalId() => Ref("unique_identifier_msgs/UUID", "goal_id");

        sb.AppendLine($"    /// <summary><c>{ros}</c>, generated from its .action definition.</summary>");
        sb.AppendLine($"    public static class {name}");
        sb.AppendLine("    {");
        sb.AppendLine("        /// <summary>ROS 2 action type name.</summary>");
        sb.AppendLine($"        public const string RosType = \"{ros}\";");
        sb.AppendLine();
        EmitMessageClass(sb, act.Goal, "Goal", $"{ros}_Goal", $"{dds}_Goal_", "The goal sent to the action server.", rootNamespace, "        ");
        sb.AppendLine();
        EmitMessageClass(sb, act.Result, "Result", $"{ros}_Result", $"{dds}_Result_", "The result returned when the goal ends.", rootNamespace, "        ");
        sb.AppendLine();
        EmitMessageClass(sb, act.Feedback, "Feedback", $"{ros}_Feedback", $"{dds}_Feedback_", "Progress published while the goal executes.", rootNamespace, "        ");

        sb.AppendLine();
        sb.AppendLine("        /// <summary>The send_goal service of the action.</summary>");
        sb.AppendLine("        public static class SendGoal");
        sb.AppendLine("        {");
        sb.AppendLine($"            public const string RosType = \"{ros}_SendGoal\";");
        sb.AppendLine();
        EmitMessageClass(sb, Synth("SendGoal_Request", GoalId(), Ref($"{act.Package}/{act.ShortName}_Goal", "goal", holder + ".Goal")),
            "Request", $"{ros}_SendGoal_Request", $"{dds}_SendGoal_Request_", "Goal id plus the goal.", rootNamespace, "            ");
        sb.AppendLine();
        EmitMessageClass(sb, Synth("SendGoal_Response", Ref("bool", "accepted"), Ref("builtin_interfaces/Time", "stamp")),
            "Response", $"{ros}_SendGoal_Response", $"{dds}_SendGoal_Response_", "Whether the goal was accepted, and when.", rootNamespace, "            ");
        sb.AppendLine("        }");

        sb.AppendLine();
        sb.AppendLine("        /// <summary>The get_result service of the action.</summary>");
        sb.AppendLine("        public static class GetResult");
        sb.AppendLine("        {");
        sb.AppendLine($"            public const string RosType = \"{ros}_GetResult\";");
        sb.AppendLine();
        EmitMessageClass(sb, Synth("GetResult_Request", GoalId()),
            "Request", $"{ros}_GetResult_Request", $"{dds}_GetResult_Request_", "The goal whose result is wanted.", rootNamespace, "            ");
        sb.AppendLine();
        EmitMessageClass(sb, Synth("GetResult_Response", Ref("int8", "status"), Ref($"{act.Package}/{act.ShortName}_Result", "result", holder + ".Result")),
            "Response", $"{ros}_GetResult_Response", $"{dds}_GetResult_Response_", "Terminal status (action_msgs/GoalStatus values) plus the result.", rootNamespace, "            ");
        sb.AppendLine("        }");

        sb.AppendLine();
        EmitMessageClass(sb, Synth("FeedbackMessage", GoalId(), Ref($"{act.Package}/{act.ShortName}_Feedback", "feedback", holder + ".Feedback")),
            "FeedbackMessage", $"{ros}_FeedbackMessage", $"{dds}_FeedbackMessage_", "What the feedback topic carries: goal id plus feedback.", rootNamespace, "        ");
        sb.AppendLine("    }");
        sb.AppendLine("}");
        return sb.ToString();
    }

    private static void FileHeader(StringBuilder sb, string rootNamespace, string package, string header)
    {
        sb.AppendLine("// <auto-generated/>");
        if (header.Length > 0) sb.AppendLine("// " + header);
        sb.AppendLine("#nullable enable");
        sb.AppendLine();
        sb.AppendLine($"namespace {rootNamespace}.{Identifier(package)}");
        sb.AppendLine("{");
    }

    private static string EmitMessageFile(MsgSpec spec, string rootNamespace, string header)
    {
        var sb = new StringBuilder();
        FileHeader(sb, rootNamespace, spec.Package, header);
        EmitMessageClass(sb, spec, Identifier(spec.ShortName),
            $"{spec.Package}/msg/{spec.ShortName}", $"{spec.Package}::msg::dds_::{spec.ShortName}_",
            $"<c>{spec.Package}/msg/{spec.ShortName}</c>, generated from its .msg definition.",
            rootNamespace, "    ");
        sb.AppendLine("}");
        return sb.ToString();
    }

    // A service is a static holder class with nested Request and Response message classes
    // (the DDS type names carry the srv module and the _Request_/_Response_ suffixes).
    private static string EmitServiceFile(SrvSpec srv, string rootNamespace, string header)
    {
        var sb = new StringBuilder();
        FileHeader(sb, rootNamespace, srv.Package, header);
        string name = Identifier(srv.ShortName);
        sb.AppendLine($"    /// <summary><c>{srv.Package}/srv/{srv.ShortName}</c>, generated from its .srv definition.</summary>");
        sb.AppendLine($"    public static class {name}");
        sb.AppendLine("    {");
        sb.AppendLine("        /// <summary>ROS 2 service type name.</summary>");
        sb.AppendLine($"        public const string RosType = \"{srv.Package}/srv/{srv.ShortName}\";");
        sb.AppendLine();
        EmitMessageClass(sb, srv.Request, "Request",
            $"{srv.Package}/srv/{srv.ShortName}_Request", $"{srv.Package}::srv::dds_::{srv.ShortName}_Request_",
            "The request half of the service.", rootNamespace, "        ");
        sb.AppendLine();
        EmitMessageClass(sb, srv.Response, "Response",
            $"{srv.Package}/srv/{srv.ShortName}_Response", $"{srv.Package}::srv::dds_::{srv.ShortName}_Response_",
            "The response half of the service.", rootNamespace, "        ");
        sb.AppendLine("    }");
        sb.AppendLine("}");
        return sb.ToString();
    }

    private static void EmitMessageClass(StringBuilder sb, MsgSpec spec, string className,
        string rosType, string ddsType, string summary, string rootNamespace, string indent)
    {
        string i1 = indent, i2 = indent + "    ", i3 = indent + "        ";
        sb.AppendLine($"{i1}/// <summary>{summary}</summary>");
        sb.AppendLine($"{i1}public sealed class {className} : global::Mori.Ros2Sharp.IRos2Message");
        sb.AppendLine($"{i1}{{");
        sb.AppendLine($"{i2}/// <summary>ROS 2 type name.</summary>");
        sb.AppendLine($"{i2}public const string RosType = \"{rosType}\";");
        sb.AppendLine($"{i2}/// <summary>DDS type name, as announced in endpoint discovery.</summary>");
        sb.AppendLine($"{i2}public const string DdsType = \"{ddsType}\";");

        foreach (MsgConstant c in spec.Constants)
            sb.AppendLine($"{i2}public const {ConstantCsType(c.Type)} {Identifier(c.Name)} = {ConstantCsValue(c)};");

        sb.AppendLine();
        foreach (MsgField f in spec.Fields)
            sb.AppendLine($"{i2}public {FieldCsType(f, rootNamespace)} {FieldName(className, f)}{FieldInitializer(f, rootNamespace)};");

        // Serialize. ROS 2 gives a message with no fields one placeholder octet on the wire
        // (std_msgs/Empty, the request of std_srvs/Trigger), so such a struct is never zero bytes.
        sb.AppendLine();
        sb.AppendLine($"{i2}/// <summary>Writes this message in wire order (CDR handles alignment).</summary>");
        sb.AppendLine($"{i2}public void Serialize(global::Mori.Ros2Sharp.CdrWriter w)");
        sb.AppendLine($"{i2}{{");
        if (spec.Fields.Count == 0)
            sb.AppendLine($"{i3}w.Write((byte)0); // placeholder member of an empty struct");
        foreach (MsgField f in spec.Fields)
            EmitSerializeField(sb, f, FieldName(className, f), i3);
        sb.AppendLine($"{i2}}}");

        // Deserialize.
        sb.AppendLine();
        sb.AppendLine($"{i2}/// <summary>Reads this message in wire order.</summary>");
        sb.AppendLine($"{i2}public void Deserialize(global::Mori.Ros2Sharp.CdrReader r)");
        sb.AppendLine($"{i2}{{");
        if (spec.Fields.Count == 0)
            sb.AppendLine($"{i3}r.ReadUInt8(); // placeholder member of an empty struct");
        foreach (MsgField f in spec.Fields)
            EmitDeserializeField(sb, f, FieldName(className, f), rootNamespace, i3);
        sb.AppendLine($"{i2}}}");

        sb.AppendLine();
        sb.AppendLine($"{i2}/// <summary>Serializes to a complete payload, CDR_LE encapsulation included.</summary>");
        sb.AppendLine($"{i2}public byte[] ToBytes() {{ var w = new global::Mori.Ros2Sharp.CdrWriter(global::Mori.Ros2Sharp.CdrEncapsulation.CdrLe); Serialize(w); return w.ToArray(); }}");
        sb.AppendLine();
        sb.AppendLine($"{i2}/// <summary>Deserializes a payload that starts with its encapsulation header.</summary>");
        sb.AppendLine($"{i2}public static {className} FromBytes(byte[] payload)");
        sb.AppendLine($"{i2}{{ var r = new global::Mori.Ros2Sharp.CdrReader(payload); var m = new {className}(); m.Deserialize(r); return m; }}");
        sb.AppendLine($"{i1}}}");
    }

    // A field whose Pascal-cased name equals the class name gets a trailing underscore
    // (C# forbids members named like their enclosing type — e.g. Clock.clock).
    private static string FieldName(string className, MsgField f)
    {
        string name = Identifier(Pascal(f.Name));
        return name == className ? name + "_" : name;
    }

    // uint8/byte/char arrays get the bulk WriteBytes/ReadBytes fast path.
    private static bool IsOctet(string baseType) => baseType == "uint8" || baseType == "byte" || baseType == "char";

    private static void EmitSerializeField(StringBuilder sb, MsgField f, string name, string indent)
    {
        string i2 = indent + "    ";
        if (!f.IsArray)
        {
            sb.AppendLine(f.IsBuiltin
                ? $"{indent}w.Write({name});"
                : $"{indent}{name}.Serialize(w);");
            return;
        }

        if (f.FixedLength < 0)
            sb.AppendLine($"{indent}w.WriteLength({name}.Length);");
        else
            sb.AppendLine($"{indent}if ({name}.Length != {f.FixedLength}) throw new global::System.InvalidOperationException(\"{f.Name} must have exactly {f.FixedLength} elements\");");

        if (f.IsBuiltin && IsOctet(f.BaseType))
        {
            sb.AppendLine($"{indent}w.WriteBytes({name});");
            return;
        }
        sb.AppendLine($"{indent}foreach (var item_{f.Name} in {name})");
        sb.AppendLine(f.IsBuiltin
            ? $"{i2}w.Write(item_{f.Name});"
            : $"{i2}item_{f.Name}.Serialize(w);");
    }

    private static void EmitDeserializeField(StringBuilder sb, MsgField f, string name, string rootNamespace, string indent)
    {
        string i2 = indent + "    ";
        if (!f.IsArray)
        {
            sb.AppendLine(f.IsBuiltin
                ? $"{indent}{name} = r.{ReadCall(f.BaseType)};"
                : $"{indent}{name}.Deserialize(r);");
            return;
        }

        if (f.IsBuiltin && IsOctet(f.BaseType))
        {
            string octetCount = f.FixedLength >= 0 ? f.FixedLength.ToString() : "r.ReadLength()";
            sb.AppendLine($"{indent}{name} = r.ReadBytes({octetCount});");
            return;
        }

        string element = ElementCsType(f, rootNamespace);
        string count = f.FixedLength >= 0 ? f.FixedLength.ToString() : "r.ReadLength()";
        sb.AppendLine($"{indent}{name} = new {element}[{count}];");
        sb.AppendLine($"{indent}for (int i_{f.Name} = 0; i_{f.Name} < {name}.Length; i_{f.Name}++)");
        sb.AppendLine(f.IsBuiltin
            ? $"{i2}{name}[i_{f.Name}] = r.{ReadCall(f.BaseType)};"
            : $"{i2}{{ var m_{f.Name} = new {element}(); m_{f.Name}.Deserialize(r); {name}[i_{f.Name}] = m_{f.Name}; }}");
    }

    // ---------------------------------------------------------------- type mapping

    private static string BuiltinCsType(string rosType) => rosType switch
    {
        "bool" => "bool",
        "int8" => "sbyte",
        "uint8" or "byte" or "char" => "byte", // all octets in ROS 2
        "int16" => "short",
        "uint16" => "ushort",
        "int32" => "int",
        "uint32" => "uint",
        "int64" => "long",
        "uint64" => "ulong",
        "float32" => "float",
        "float64" => "double",
        "string" => "string",
        "wstring" => throw new NotSupportedException("wstring fields are not supported"),
        _ => throw new ArgumentException($"not a builtin: {rosType}"),
    };

    private static string ElementCsType(MsgField f, string rootNamespace)
    {
        if (f.CsType != null) return f.CsType;
        if (f.IsBuiltin) return BuiltinCsType(f.BaseType);
        int slash = f.BaseType.IndexOf('/');
        return $"global::{rootNamespace}.{Identifier(f.BaseType.Substring(0, slash))}.{Identifier(f.BaseType.Substring(slash + 1))}";
    }

    private static string FieldCsType(MsgField f, string rootNamespace) =>
        f.IsArray ? ElementCsType(f, rootNamespace) + "[]" : ElementCsType(f, rootNamespace);

    private static string FieldInitializer(MsgField f, string rootNamespace)
    {
        if (f.IsArray)
            return f.FixedLength >= 0
                ? $" = new {ElementCsType(f, rootNamespace)}[{f.FixedLength}]"
                : $" = global::System.Array.Empty<{ElementCsType(f, rootNamespace)}>()";
        if (!f.IsBuiltin) return $" = new {ElementCsType(f, rootNamespace)}()";

        // Scalar defaults from the field line ("bool flag true", "string label \"hi\"").
        if (f.DefaultText is string def && def.Length > 0)
        {
            if (f.BaseType == "string") return $" = \"{Escape(Unquote(def))}\"";
            if (f.BaseType == "bool")
                return def == "1" || string.Equals(def, "true", StringComparison.OrdinalIgnoreCase) ? " = true" : " = false";
            if (f.BaseType == "float32") return $" = {def}f";
            return $" = {def}";
        }
        return f.BaseType == "string" ? " = \"\"" : "";
    }

    private static string ReadCall(string builtin) => builtin switch
    {
        "bool" => "ReadBool()",
        "int8" => "ReadInt8()",
        "uint8" or "byte" or "char" => "ReadUInt8()",
        "int16" => "ReadInt16()",
        "uint16" => "ReadUInt16()",
        "int32" => "ReadInt32()",
        "uint32" => "ReadUInt32()",
        "int64" => "ReadInt64()",
        "uint64" => "ReadUInt64()",
        "float32" => "ReadFloat32()",
        "float64" => "ReadFloat64()",
        "string" => "ReadString()",
        _ => throw new ArgumentException($"not a builtin: {builtin}"),
    };

    private static string ConstantCsType(string rosType) =>
        rosType == "string" ? "string" : BuiltinCsType(rosType);

    private static string ConstantCsValue(MsgConstant c)
    {
        if (c.Type == "string") return $"\"{Escape(Unquote(c.ValueText))}\"";
        if (c.Type == "bool")
            return c.ValueText == "1" || string.Equals(c.ValueText, "true", StringComparison.OrdinalIgnoreCase) ? "true" : "false";
        if (c.Type == "float32") return c.ValueText + "f";
        return c.ValueText;
    }

    /// <summary>Strips one pair of matching single or double quotes, if present.</summary>
    private static string Unquote(string s) =>
        s.Length >= 2 && (s[0] == '"' || s[0] == '\'') && s[s.Length - 1] == s[0]
            ? s.Substring(1, s.Length - 2)
            : s;

    private static string Escape(string s) => s.Replace("\\", "\\\\").Replace("\"", "\\\"");

    // ---------------------------------------------------------------- identifiers

    /// <summary>snake_case → PascalCase (ROS field convention → C# convention).</summary>
    public static string Pascal(string snake)
    {
        var sb = new StringBuilder(snake.Length);
        bool upper = true;
        foreach (char ch in snake)
        {
            if (ch == '_') { upper = true; continue; }
            sb.Append(upper ? char.ToUpperInvariant(ch) : ch);
            upper = false;
        }
        return sb.ToString();
    }

    private static readonly HashSet<string> Keywords = new HashSet<string>
    {
        "abstract", "as", "base", "bool", "break", "byte", "case", "catch", "char", "checked",
        "class", "const", "continue", "decimal", "default", "delegate", "do", "double", "else",
        "enum", "event", "explicit", "extern", "false", "finally", "fixed", "float", "for",
        "foreach", "goto", "if", "implicit", "in", "int", "interface", "internal", "is", "lock",
        "long", "namespace", "new", "null", "object", "operator", "out", "override", "params",
        "private", "protected", "public", "readonly", "ref", "return", "sbyte", "sealed", "short",
        "sizeof", "stackalloc", "static", "string", "struct", "switch", "this", "throw", "true",
        "try", "typeof", "uint", "ulong", "unchecked", "unsafe", "ushort", "using", "virtual",
        "void", "volatile", "while",
    };

    /// <summary>Escapes C# keywords and invalid identifier characters.</summary>
    public static string Identifier(string name)
    {
        var sb = new StringBuilder(name.Length);
        foreach (char ch in name)
            sb.Append(char.IsLetterOrDigit(ch) || ch == '_' ? ch : '_');
        string id = sb.Length == 0 || char.IsDigit(sb[0]) ? "_" + sb : sb.ToString();
        return Keywords.Contains(id) ? "@" + id : id;
    }
}
