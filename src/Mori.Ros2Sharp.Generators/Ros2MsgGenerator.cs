using System.Collections.Immutable;
using Microsoft.CodeAnalysis;
using Mori.Ros2Sharp.Msg;

namespace Mori.Ros2Sharp.Generators;

/// <summary>
/// Generates typed C# classes from .msg and .srv files included as AdditionalFiles. The
/// package name comes from the file's directory ROS-style (<c>&lt;package&gt;/msg/Name.msg</c>,
/// <c>&lt;package&gt;/srv/Name.srv</c>, or <c>&lt;package&gt;/Name.msg</c>); nested types resolve
/// against the other AdditionalFiles first, then the embedded common set, and dependencies
/// are emitted too so the code always compiles. The root namespace defaults to
/// <c>Ros2Messages</c> and can be overridden with the <c>Ros2SharpNamespace</c> MSBuild property.
/// </summary>
[Generator]
public sealed class Ros2MsgGenerator : IIncrementalGenerator
{
    private static readonly DiagnosticDescriptor ParseError = new(
        id: "ROS2MSG001",
        title: "Invalid .msg file",
        messageFormat: "{0}: {1}",
        category: "Mori.Ros2Sharp",
        DiagnosticSeverity.Error,
        isEnabledByDefault: true);

    public void Initialize(IncrementalGeneratorInitializationContext context)
    {
        IncrementalValueProvider<ImmutableArray<(string FullType, string Text, string Path)>> msgFiles =
            context.AdditionalTextsProvider
                .Where(static f => f.Path.EndsWith(".msg", StringComparison.OrdinalIgnoreCase) ||
                                   f.Path.EndsWith(".srv", StringComparison.OrdinalIgnoreCase))
                .Select(static (f, ct) => (FullTypeOf(f.Path), f.GetText(ct)?.ToString() ?? "", f.Path))
                .Collect();

        IncrementalValueProvider<string> rootNamespace =
            context.AnalyzerConfigOptionsProvider.Select(static (options, _) =>
                options.GlobalOptions.TryGetValue("build_property.Ros2SharpNamespace", out string? ns)
                && !string.IsNullOrWhiteSpace(ns) ? ns! : "Ros2Messages");

        context.RegisterSourceOutput(msgFiles.Combine(rootNamespace), static (spc, input) =>
        {
            (ImmutableArray<(string FullType, string Text, string Path)> files, string ns) = input;
            if (files.Length == 0) return;

            var messages = new Dictionary<string, string>();
            var services = new Dictionary<string, string>();
            foreach ((string fullType, string text, string path) in files)
                (path.EndsWith(".srv", StringComparison.OrdinalIgnoreCase) ? services : messages)[fullType] = text;

            var catalog = new MsgCatalog(
                fullType => messages.TryGetValue(fullType, out string? text) ? text : null,
                fullType => services.TryGetValue(fullType, out string? text) ? text : null);

            try
            {
                foreach (KeyValuePair<string, string> generated in
                         CSharpEmitter.EmitClosure(messages.Keys, services.Keys.Concat(EmbeddedMessages.ServiceTypes), catalog, ns, ""))
                    spc.AddSource(generated.Key.Replace('/', '.') + ".g.cs", generated.Value);
            }
            catch (Exception ex)
            {
                spc.ReportDiagnostic(Diagnostic.Create(ParseError, Location.None, files[0].Path, ex.Message));
            }
        });
    }

    // ROS layout: <package>/msg/<Name>.msg or <package>/srv/<Name>.srv — the package is the
    // directory above "msg"/"srv".
    private static string FullTypeOf(string path)
    {
        string name = Path.GetFileNameWithoutExtension(path);
        string? dir = Path.GetDirectoryName(path);
        string package = "msgs";
        if (dir is not null)
        {
            string leaf = Path.GetFileName(dir);
            if (string.Equals(leaf, "msg", StringComparison.OrdinalIgnoreCase) ||
                string.Equals(leaf, "srv", StringComparison.OrdinalIgnoreCase))
            {
                string? parent = Path.GetDirectoryName(dir);
                if (parent is not null) leaf = Path.GetFileName(parent);
            }
            if (!string.IsNullOrEmpty(leaf)) package = leaf;
        }
        return package + "/" + name;
    }
}
