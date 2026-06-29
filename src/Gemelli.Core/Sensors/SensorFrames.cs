namespace Gemelli.Core.Sensors;

/// <summary>Scalar element type of a render variable buffer (mirrors the DLPack type codes we care about).</summary>
public enum ScalarType : byte
{
    Int = 0,
    UInt = 1,
    Float = 2,
    Bool = 6,
    Other = 255,
}

/// <summary>
/// A single render variable (e.g. <c>LdrColor</c>, <c>Depth</c>, a lidar point cloud) copied out of
/// native/GPU memory into a managed byte buffer. Wrapper-free so it can cross the process boundary.
/// Shape and element type are preserved so callers can reinterpret the bytes (RGBA8, float32 depth, …).
/// </summary>
public sealed record RenderVarData(
    string Name,
    long[] Shape,
    ScalarType ElementType,
    byte ElementBits,
    ushort Lanes,
    byte[] Bytes)
{
    /// <summary>Total scalar elements (product of <see cref="Shape"/>).</summary>
    public long ElementCount
    {
        get
        {
            long c = 1;
            foreach (long d in Shape) c *= d;
            return c;
        }
    }

    /// <summary>
    /// Convenience accessors for the image case. ovrtx returns color as a 2D <c>[h, w]</c> shape with
    /// the channels packed into <see cref="Lanes"/> (e.g. RGBA8 = UInt8 × 4 lanes); some vars instead
    /// carry channels as a 3rd shape dimension. Both are handled here.
    /// </summary>
    public int Height => Shape.Length >= 1 ? (int)Shape[0] : 0;
    public int Width => Shape.Length >= 2 ? (int)Shape[1] : 0;
    public int Channels => Shape.Length >= 3 ? (int)Shape[2] : Math.Max(1, (int)Lanes);
}

/// <summary>All render variables produced for one render product at one simulation frame.</summary>
public sealed record CapturedFrame(
    string RenderProduct,
    double StartTime,
    double EndTime,
    IReadOnlyDictionary<string, RenderVarData> Vars)
{
    // Render-var names vary by how the product was authored (Isaac Sim appends an "SD" suffix on the
    // SyntheticData pipeline outputs). Match the known aliases for each logical channel, in preference order.
    private static readonly string[] ColorNames = { "LdrColor", "LdrColorSD", "HdrColor", "HdrColorSD" };
    private static readonly string[] DepthNames = { "DistanceToImagePlaneSD", "DistanceToImagePlane", "Depth", "DepthLinearized", "DistanceToCameraSD" };
    private static readonly string[] SegmentationNames = { "InstanceSegmentationSD", "SemanticSegmentationSD", "InstanceSegmentation", "SemanticSegmentation", "InstanceIdSegmentationSD" };

    /// <summary>Returns the first present var among <paramref name="names"/> (alias preference order), or null.</summary>
    private RenderVarData? First(string[] names)
    {
        foreach (string n in names)
            if (Vars.TryGetValue(n, out var v)) return v;
        return null;
    }

    /// <summary>The color image (<c>LdrColor</c>/<c>LdrColorSD</c> preferred), or null.</summary>
    public RenderVarData? Color => First(ColorNames);

    /// <summary>The per-pixel depth (distance-to-image-plane), or null if the product carries none.</summary>
    public RenderVarData? Depth => First(DepthNames);

    /// <summary>The per-pixel segmentation (instance/semantic id), or null if the product carries none.</summary>
    public RenderVarData? Segmentation => First(SegmentationNames);
}
