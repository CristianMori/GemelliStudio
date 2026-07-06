namespace Gemelli.Fmi;

/// <summary>Where a signal touches the scene: a USD attribute or physics routing on a target prim,
/// with the component selector from the schema ((offset, count); (0,0) = scalar).</summary>
public sealed record SignalEndpoint(string TargetPath, string UsdAttribute, int Offset, int Count)
{
    /// <summary>Short human label, e.g. "Roller_00/DriveJoint_Zone00" or "ConveyorControls.translate[0]".</summary>
    public string Label
    {
        get
        {
            string prim = TargetPath.TrimEnd('/').Split('/')[^1];
            string attr = UsdAttribute.Split(':')[^1];
            return FmiSchema.IsPhysicsAttribute(UsdAttribute) || Count == 0
                ? $"{prim}.{attr}"
                : $"{prim}.{attr}[{Offset}]";
        }
    }
}

/// <summary>
/// One live wire between the scene and an FMI model variable. Mutable so the signal-mapper UI can
/// observe values and rewire at runtime: <see cref="LastValue"/> is written on the sim thread each
/// frame (torn reads are impossible on x64; the UI only displays it).
/// </summary>
public sealed class SignalMapping
{
    public required int Id { get; init; }
    public required string InstancePath { get; init; }
    public required string FmuVariable { get; init; }
    /// <summary>true: scene → FMU input. false: FMU output → scene.</summary>
    public required bool IsInput { get; init; }
    public required SignalEndpoint Endpoint { get; init; }

    /// <summary>The value that crossed this wire on the most recent step.</summary>
    public double LastValue;
}

/// <summary>An FMI instance's connectable surface: its input and output variable names.</summary>
public sealed record FmiInstancePorts(
    string PrimPath, string Name, bool IsSsp,
    IReadOnlyList<string> Inputs, IReadOnlyList<string> Outputs);

/// <summary>
/// A user-defined constant signal source, wireable to FMU inputs or directly to actuators.
/// <see cref="Value"/> is written by the UI and read by the sim thread each frame (atomic on x64),
/// so edits take effect on the next step.
/// </summary>
public sealed class FmiConstant
{
    public required int Id { get; init; }
    public required string Name { get; set; }
    public double Value;

    /// <summary>The pseudo target path constant wires carry in their endpoint.</summary>
    public string Path => $"const:{Id}";
}
