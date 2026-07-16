# ros2msggen

Generates typed C# message classes from ROS 2 `.msg` files, for use with
[Mori.Ros2Sharp](https://www.nuget.org/packages/Mori.Ros2Sharp) — serialization is OMG CDR,
the wire format of every ROS 2 message.

```
dotnet tool install --global Mori.Ros2Sharp.MsgGen
ros2msggen -o Generated -n MyMessages path/to/my_package/msg
```

Inputs are `.msg` files or directories laid out ROS-style (`<package>/msg/<Name>.msg`).
Nested types resolve against the inputs first, then an embedded copy of the common interface
packages (`builtin_interfaces`, `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`,
`tf2_msgs`), and the emitted set is closed over dependencies, so the output always compiles.

| Option | Meaning |
|---|---|
| `-o <dir>` | output directory for the generated `.cs` files (required) |
| `-n <namespace>` | root namespace (default `Ros2Messages`); classes land in `<ns>.<package>` |
| `--package <name>` | package name for loose `.msg` files whose directory doesn't imply one |

Each generated class carries `RosType` / `DdsType` constants, applies `.msg` field defaults,
and provides `Serialize`/`Deserialize` plus `ToBytes`/`FromBytes` (CDR encapsulation
included).

If you reference `Mori.Ros2Sharp` from an SDK-style project you usually don't need this tool:
the same generator ships inside that package and runs at build time on any `.msg` files added
as `AdditionalFiles`. This tool is for everything else — generated-code review, non-SDK
builds, or checking generated sources into a repository.

License: Apache-2.0 — © 2026 Cristian Mori
