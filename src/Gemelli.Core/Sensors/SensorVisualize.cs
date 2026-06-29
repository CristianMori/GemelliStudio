namespace Gemelli.Core.Sensors;

/// <summary>What a sensor view should display.</summary>
public enum SensorChannel
{
    Color = 0,
    Depth = 1,
    Segmentation = 2,
}

/// <summary>
/// Turns raw render-variable buffers into a displayable/recordable RGBA8 image: color passes through,
/// depth is normalized to grayscale (near = bright), segmentation ids are colorized. Pure and
/// dependency-free so both the Studio viewport and the headless recorder reuse it.
/// </summary>
public static class SensorVisualize
{
    /// <summary>
    /// Produce an 8-bit RGBA image (row-major, <c>w*h*4</c> bytes) for the requested channel, or null if
    /// that channel isn't present in the frame. Width/height come from the underlying render var.
    /// </summary>
    public static (int Width, int Height, byte[] Rgba)? Render(CapturedFrame frame, SensorChannel channel) => channel switch
    {
        SensorChannel.Color => ColorRgba(frame.Color),
        SensorChannel.Depth => DepthGray(frame.Depth),
        SensorChannel.Segmentation => SegmentationColor(frame.Segmentation),
        _ => null,
    };

    /// <summary>Color render var (RGBA8 or RGB8) widened to RGBA8.</summary>
    public static (int, int, byte[])? ColorRgba(RenderVarData? color)
    {
        if (color is null || color.Width == 0 || color.Height == 0) return null;
        int w = color.Width, h = color.Height, ch = color.Channels;
        if (ch is not (3 or 4)) return null;
        var dst = new byte[w * h * 4];
        ReadOnlySpan<byte> s = color.Bytes;
        for (int i = 0, p = 0, q = 0; i < w * h; i++, p += ch, q += 4)
        {
            dst[q] = s[p]; dst[q + 1] = s[p + 1]; dst[q + 2] = s[p + 2];
            dst[q + 3] = ch == 4 ? s[p + 3] : (byte)255;
        }
        return (w, h, dst);
    }

    /// <summary>
    /// Depth → grayscale. The buffer is interpreted as float32 if its element type is float, else uint32;
    /// background (non-finite / zero / saturated) is rendered black, and the finite range is normalized so
    /// near surfaces are bright. Returns null if no finite depth samples exist.
    /// </summary>
    public static (int, int, byte[])? DepthGray(RenderVarData? depth)
    {
        if (depth is null || depth.Width == 0 || depth.Height == 0) return null;
        int w = depth.Width, h = depth.Height, n = w * h;
        // DistanceToImagePlane is float32 distance even when the SyntheticData GPU-interop copy tags the
        // buffer as UInt32 (background = +inf). Bit-reinterpret as float32 when there are 4 bytes/pixel.
        float[] d = depth.ElementBits >= 32 ? ReadFloat32(depth, n) : ReadScalar(depth, n);
        if (d.Length < n) return null;

        // Find the finite, positive range (ignore background sentinels: 0, inf, NaN, and huge values).
        float min = float.PositiveInfinity, max = float.NegativeInfinity;
        for (int i = 0; i < n; i++)
        {
            float v = d[i];
            if (!float.IsFinite(v) || v <= 0f || v >= 1e6f) continue;
            if (v < min) min = v;
            if (v > max) max = v;
        }
        if (min > max) return null; // nothing finite to show
        float range = MathF.Max(max - min, 1e-6f);

        var dst = new byte[n * 4];
        for (int i = 0, q = 0; i < n; i++, q += 4)
        {
            float v = d[i];
            byte g;
            if (!float.IsFinite(v) || v <= 0f || v >= 1e6f) g = 0; // background
            else g = (byte)(255f * (1f - (v - min) / range));      // near = bright
            dst[q] = g; dst[q + 1] = g; dst[q + 2] = g; dst[q + 3] = 255;
        }
        return (w, h, dst);
    }

    /// <summary>Segmentation ids → distinct colors (id 0 = background = black).</summary>
    public static (int, int, byte[])? SegmentationColor(RenderVarData? seg)
    {
        if (seg is null || seg.Width == 0 || seg.Height == 0) return null;
        int w = seg.Width, h = seg.Height, n = w * h;
        float[] ids = ReadScalar(seg, n);
        if (ids.Length < n) return null;

        var dst = new byte[n * 4];
        for (int i = 0, q = 0; i < n; i++, q += 4)
        {
            uint id = (uint)MathF.Max(0f, ids[i]);
            if (id == 0) { dst[q + 3] = 255; continue; } // background black
            var (r, g, b) = Palette(id);
            dst[q] = r; dst[q + 1] = g; dst[q + 2] = b; dst[q + 3] = 255;
        }
        return (w, h, dst);
    }

    /// <summary>Bit-reinterprets a 4-byte-per-pixel render var as float32 (used for depth).</summary>
    private static float[] ReadFloat32(RenderVarData v, int count)
    {
        ReadOnlySpan<byte> b = v.Bytes;
        if (b.Length < (long)count * 4) count = b.Length / 4;
        var outv = new float[count];
        for (int i = 0; i < count; i++) outv[i] = BitConverter.ToSingle(b.Slice(i * 4, 4));
        return outv;
    }

    /// <summary>Reads a single-lane render var as float32 regardless of stored type (uint/int/float).</summary>
    private static float[] ReadScalar(RenderVarData v, int count)
    {
        var outv = new float[count];
        ReadOnlySpan<byte> b = v.Bytes;
        int bytesPer = Math.Max(1, v.ElementBits / 8);
        if (b.Length < (long)count * bytesPer) count = b.Length / bytesPer;
        for (int i = 0; i < count; i++)
        {
            int o = i * bytesPer;
            outv[i] = v.ElementType switch
            {
                ScalarType.Float when bytesPer >= 4 => BitConverter.ToSingle(b.Slice(o, 4)),
                ScalarType.UInt when bytesPer >= 4 => BitConverter.ToUInt32(b.Slice(o, 4)),
                ScalarType.UInt when bytesPer == 2 => BitConverter.ToUInt16(b.Slice(o, 2)),
                ScalarType.Int when bytesPer >= 4 => BitConverter.ToInt32(b.Slice(o, 4)),
                _ => bytesPer >= 4 ? BitConverter.ToUInt32(b.Slice(o, 4)) : b[o],
            };
        }
        return outv;
    }

    // Golden-ratio hue hashing gives well-separated, stable colors per id.
    private static (byte, byte, byte) Palette(uint id)
    {
        float hue = (id * 0.61803398875f) % 1f;
        return HsvToRgb(hue, 0.65f, 0.95f);
    }

    // Standard HSV→RGB (h,s,v in 0..1) yielding 8-bit channels.
    private static (byte, byte, byte) HsvToRgb(float h, float s, float v)
    {
        float i = MathF.Floor(h * 6f);
        float f = h * 6f - i;
        float p = v * (1f - s), q = v * (1f - f * s), t = v * (1f - (1f - f) * s);
        (float r, float g, float b) = ((int)i % 6) switch
        {
            0 => (v, t, p), 1 => (q, v, p), 2 => (p, v, t),
            3 => (p, q, v), 4 => (t, p, v), _ => (v, p, q),
        };
        return ((byte)(r * 255), (byte)(g * 255), (byte)(b * 255));
    }
}
