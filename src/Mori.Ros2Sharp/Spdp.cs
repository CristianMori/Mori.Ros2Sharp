namespace Mori.Ros2Sharp;

/// <summary>
/// Everything a participant announces about itself in SPDP — the payload of the periodic
/// DATA(p) written by the builtin participant writer.
/// </summary>
public sealed class ParticipantData
{
    public RtpsGuid Guid { get; set; }
    public ProtocolVersion ProtocolVersion { get; set; } = ProtocolVersion.Local;
    public VendorId VendorId { get; set; } = VendorId.Local;
    public BuiltinEndpoints BuiltinEndpoints { get; set; }
    public RtpsDuration LeaseDuration { get; set; } = RtpsDuration.FromSeconds(20);
    public int? DomainId { get; set; }
    public string? EntityName { get; set; }
    public byte[]? UserData { get; set; }

    public List<Locator> MetatrafficUnicastLocators { get; } = new();
    public List<Locator> MetatrafficMulticastLocators { get; } = new();
    public List<Locator> DefaultUnicastLocators { get; } = new();
    public List<Locator> DefaultMulticastLocators { get; } = new();

    /// <summary>Local bookkeeping (lease tracking) — not part of the wire format.</summary>
    public DateTimeOffset LastSeen { get; set; }

    /// <summary>The PL_CDR_LE payload of our periodic DATA(p) announcement.</summary>
    public byte[] Encode()
    {
        var p = new ParameterListWriter();
        p.Add(Pid.ProtocolVersion, ProtocolVersion);
        p.Add(Pid.VendorId, VendorId);
        p.Add(Pid.ParticipantGuid, Guid);
        p.Add(Pid.BuiltinEndpointSet, (uint)BuiltinEndpoints);
        if (DomainId is int domain) p.Add(Pid.DomainId, (uint)domain);
        foreach (var l in MetatrafficUnicastLocators) p.Add(Pid.MetatrafficUnicastLocator, l);
        foreach (var l in MetatrafficMulticastLocators) p.Add(Pid.MetatrafficMulticastLocator, l);
        foreach (var l in DefaultUnicastLocators) p.Add(Pid.DefaultUnicastLocator, l);
        foreach (var l in DefaultMulticastLocators) p.Add(Pid.DefaultMulticastLocator, l);
        p.Add(Pid.ParticipantLeaseDuration, LeaseDuration);
        if (EntityName is { Length: > 0 }) p.Add(Pid.EntityName, EntityName);
        if (UserData is { Length: > 0 }) p.AddOctetSequence(Pid.UserData, UserData);
        return p.Finish();
    }

    /// <summary>
    /// Decodes a remote announcement. Unknown parameter ids are skipped by design — vendors
    /// add many. Requires a participant guid to consider the data valid.
    /// </summary>
    public static bool TryDecode(byte[] payload, out ParticipantData? data)
    {
        data = null;
        if (payload.Length < 4 || payload[0] != 0x00 || payload[1] != 0x03) return false; // PL_CDR_LE only

        var d = new ParticipantData();
        var p = new ParameterListReader(payload, 0, payload.Length);
        while (p.Next(out ushort pid, out var v))
        {
            var r = new CdrReader(v.Array!, v.Offset, v.Count, hasEncapsulation: false);
            switch (pid)
            {
                case Pid.ProtocolVersion when v.Count >= 2:
                    d.ProtocolVersion = new ProtocolVersion(v[0], v[1]);
                    break;
                case Pid.VendorId when v.Count >= 2:
                    d.VendorId = new VendorId(v[0], v[1]);
                    break;
                case Pid.ParticipantGuid when v.Count >= 16:
                    d.Guid = RtpsGuid.ReadFrom(v.AsSpan());
                    break;
                case Pid.BuiltinEndpointSet when v.Count >= 4:
                    d.BuiltinEndpoints = (BuiltinEndpoints)r.ReadUInt32();
                    break;
                case Pid.DomainId when v.Count >= 4:
                    d.DomainId = (int)r.ReadUInt32();
                    break;
                case Pid.ParticipantLeaseDuration when v.Count >= 8:
                    d.LeaseDuration = new RtpsDuration(r.ReadInt32(), r.ReadUInt32());
                    break;
                case Pid.MetatrafficUnicastLocator when v.Count >= 24:
                    d.MetatrafficUnicastLocators.Add(Locator.ReadFrom(r));
                    break;
                case Pid.MetatrafficMulticastLocator when v.Count >= 24:
                    d.MetatrafficMulticastLocators.Add(Locator.ReadFrom(r));
                    break;
                case Pid.DefaultUnicastLocator when v.Count >= 24:
                    d.DefaultUnicastLocators.Add(Locator.ReadFrom(r));
                    break;
                case Pid.DefaultMulticastLocator when v.Count >= 24:
                    d.DefaultMulticastLocators.Add(Locator.ReadFrom(r));
                    break;
                case Pid.EntityName when v.Count >= 4:
                    d.EntityName = r.ReadString();
                    break;
                case Pid.UserData when v.Count >= 4:
                    d.UserData = r.ReadBytes(Math.Min(r.ReadLength(), r.Remaining));
                    break;
            }
        }

        if (d.Guid.Prefix == default) return false;
        data = d;
        return true;
    }
}
