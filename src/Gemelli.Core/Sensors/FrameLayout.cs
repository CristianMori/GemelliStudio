using Gemelli.Core.Ipc;

namespace Gemelli.Core.Sensors;

/// <summary>Where one render variable's pixels live in the shared <see cref="FrameBuffer"/>.</summary>
public sealed record VarLayout(
    string Name, long[] Shape, ScalarType ElementType, byte ElementBits, ushort Lanes, long Offset, int Length);

/// <summary>One render product's frame, described by shared-buffer offsets (no pixels inline).</summary>
public sealed record FrameLayout(string RenderProduct, double StartTime, double EndTime, IReadOnlyList<VarLayout> Vars);

/// <summary>
/// Serializes <see cref="FrameLayout"/> metadata across the render pipe. Pixel bytes travel through the
/// shared <see cref="FrameBuffer"/>; only these small descriptors go over the wire.
/// </summary>
public static class FrameLayoutCodec
{
    /// <summary>Writes the frame descriptors and per-var shared-buffer offset/length pairs (no pixels).</summary>
    public static void Write(BinaryWriter w, IReadOnlyList<FrameLayout> frames)
    {
        w.Write(frames.Count);
        foreach (FrameLayout f in frames)
        {
            w.Write(f.RenderProduct);
            w.Write(f.StartTime);
            w.Write(f.EndTime);
            w.Write(f.Vars.Count);
            foreach (VarLayout v in f.Vars)
            {
                w.Write(v.Name);
                Wire.WriteLongArray(w, v.Shape);
                w.Write((byte)v.ElementType);
                w.Write(v.ElementBits);
                w.Write(v.Lanes);
                w.Write(v.Offset);
                w.Write(v.Length);
            }
        }
    }

    /// <summary>Reads the layout and materializes frames by copying pixels out of <paramref name="buffer"/>.</summary>
    public static List<CapturedFrame> ReadAndMaterialize(BinaryReader r, FrameBuffer buffer)
    {
        int frameCount = r.ReadInt32();
        var frames = new List<CapturedFrame>(frameCount);
        for (int i = 0; i < frameCount; i++)
        {
            string product = r.ReadString();
            double start = r.ReadDouble();
            double end = r.ReadDouble();
            int varCount = r.ReadInt32();
            var vars = new Dictionary<string, RenderVarData>(varCount);
            for (int v = 0; v < varCount; v++)
            {
                string name = r.ReadString();
                long[] shape = Wire.ReadLongArray(r);
                var type = (ScalarType)r.ReadByte();
                byte bits = r.ReadByte();
                ushort lanes = r.ReadUInt16();
                long offset = r.ReadInt64();
                int length = r.ReadInt32();
                byte[] bytes = buffer.Read(offset, length); // copy out of shared memory
                vars[name] = new RenderVarData(name, shape, type, bits, lanes, bytes);
            }
            frames.Add(new CapturedFrame(product, start, end, vars));
        }
        return frames;
    }
}
