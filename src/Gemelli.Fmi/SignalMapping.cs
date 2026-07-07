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

/// <summary>A block output pin acting as a wire's source, or a block input pin as its sink.</summary>
public sealed record PinRef(string BlockPath, string Pin);

/// <summary>
/// One live wire in the signal graph. The source is EITHER a scene endpoint (sensor, USD attribute,
/// constant) OR a block output pin; the sink is EITHER a block input pin OR a scene endpoint
/// (actuator). Element offsets select components when the two sides have different widths (a thick
/// wire dropped on a scalar pin). Mutable so the mapper can observe values and rewire at runtime:
/// <see cref="LastValues"/> is swapped on the sim thread each frame (the UI only displays it).
/// </summary>
public sealed class SignalMapping
{
    public required int Id { get; init; }

    // Exactly one of the two sources and one of the two sinks is non-null.
    public SignalEndpoint? SourceEndpoint { get; init; }
    public PinRef? SourcePin { get; init; }
    public PinRef? SinkPin { get; init; }
    public SignalEndpoint? SinkEndpoint { get; init; }

    /// <summary>First element carried from the source (into a vector source pin).</summary>
    public int SourceOffset { get; init; }
    /// <summary>First element written at the sink (into a vector sink pin).</summary>
    public int SinkOffset { get; init; }
    /// <summary>How many elements this wire carries (1 = scalar wire).</summary>
    public int Count { get; init; } = 1;

    /// <summary>The value(s) that crossed this wire on the most recent step.</summary>
    public double[] LastValues = [];

    /// <summary>Convenience: the scalar value for display on thin wires (0 when empty).</summary>
    public double LastValue => LastValues.Length > 0 ? LastValues[0] : 0;
}

/// <summary>An FMI instance's connectable surface: its input and output variable names.</summary>
public sealed record FmiInstancePorts(
    string PrimPath, string Name, bool IsSsp,
    IReadOnlyList<string> Inputs, IReadOnlyList<string> Outputs);

/// <summary>
/// A user-defined constant signal source, wireable to block inputs or directly to actuators.
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
