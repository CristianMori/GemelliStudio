using Gemelli.Core.Ipc;

namespace Gemelli.Core.Sensors;

/// <summary>Serializes <see cref="CapturedFrame"/> sets across the render-worker pipe.</summary>
public static class FrameCodec
{
    /// <summary>Writes each frame's metadata followed by every render var's shape, dtype and inline pixel bytes.</summary>
    public static void Write(BinaryWriter w, IReadOnlyList<CapturedFrame> frames)
    {
        w.Write(frames.Count);
        foreach (CapturedFrame frame in frames)
        {
            w.Write(frame.RenderProduct);
            w.Write(frame.StartTime);
            w.Write(frame.EndTime);
            w.Write(frame.Vars.Count);
            foreach (var (_, v) in frame.Vars)
            {
                w.Write(v.Name);
                Wire.WriteLongArray(w, v.Shape);
                w.Write((byte)v.ElementType);
                w.Write(v.ElementBits);
                w.Write(v.Lanes);
                Wire.WriteBytes(w, v.Bytes);
            }
        }
    }

    /// <summary>Inverse of <see cref="Write"/>: reconstructs the frames and their render vars from the stream.</summary>
    public static List<CapturedFrame> Read(BinaryReader r)
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
                var elemType = (ScalarType)r.ReadByte();
                byte bits = r.ReadByte();
                ushort lanes = r.ReadUInt16();
                byte[] bytes = Wire.ReadBytes(r);
                vars[name] = new RenderVarData(name, shape, elemType, bits, lanes, bytes);
            }
            frames.Add(new CapturedFrame(product, start, end, vars));
        }
        return frames;
    }
}
