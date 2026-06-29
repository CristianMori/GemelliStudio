using System.Buffers.Binary;

namespace Gemelli.Core.Ipc;

/// <summary>Opcodes for the physics worker (ovphysx host).</summary>
public enum PhysicsOp : ushort
{
    Init = 1,            // (int device) -> ()
    LoadUsd = 2,         // (string path) -> ()
    BindPoses = 3,       // (string pattern) -> (string[] primPaths)
    StepAndReadPoses = 4,// (float dt, float simTime) -> (int n, float[n*7])
    WriteTensor = 5,     // (int tensorType, string pattern, float[] values) -> ()
    ReadTensor = 6,      // (int tensorType, string pattern) -> (long[] shape, float[] data)
    ReadDofNames = 7,    // (string pattern) -> (string[] dofNames)
    Shutdown = 255,      // () -> ()
}

/// <summary>
/// Tensor channels exposed to controllers via <c>ISimApi</c> (subset of ovphysx tensor types;
/// values mirror <c>Nvidia.OvPhysx.TensorType</c> so the worker can cast directly).
/// </summary>
public enum SimTensor
{
    RigidBodyPose = 1,            // [N, 7] (px,py,pz, qx,qy,qz,qw)
    RigidBodyVelocity = 2,        // [N, 6] (lin xyz, ang xyz)
    ArticulationLinkPose = 20,    // [N, L, 7] world pose per link (read-only)
    ArticulationDofPosition = 30, // [N, D]
    ArticulationDofVelocity = 31, // [N, D]
    ArticulationDofPositionTarget = 32, // [N, D]
    ArticulationDofVelocityTarget = 33, // [N, D]
    ArticulationDofActuationForce = 34, // [N, D]
    ArticulationJacobian = 70,    // [N, R, C] R=6*movingLinks, C=D(+6 floating base) (read-only)
}

/// <summary>Opcodes for the render worker (ovrtx host).</summary>
public enum RenderOp : ushort
{
    Init = 1,            // (string ovrtxDir, bool syncMode, string frameBufName, long frameBufCap) -> (uint maj,min,patch)
    LoadUsd = 2,         // (string path) -> ()
    Warmup = 3,          // (string[] products, int frames) -> ()
    WriteAndStep = 4,    // (string[] paths, double[] matrices, string[] products, double dt) -> (bool sharedMem, frames)
    Shutdown = 255,      // () -> ()
}

/// <summary>Thrown when a worker reports a handled error across the pipe.</summary>
public class TwinWorkerException : Exception
{
    public TwinWorkerException(string message) : base(message) { }
}

/// <summary>
/// Thrown when a worker closes the pipe without replying — i.e. it crashed or exited mid-call,
/// as opposed to reporting a handled error. <c>WorkerHost</c> catches this to attach the worker's
/// name, exit code, and log tail.
/// </summary>
public sealed class TwinWorkerDisconnectedException : TwinWorkerException
{
    public TwinWorkerDisconnectedException(string message) : base(message) { }
}

/// <summary>Length-prefixed framing over a stream: <c>[int32 length][payload]</c> (little-endian).</summary>
public static class Frame
{
    /// <summary>Writes one frame: the 4-byte little-endian length prefix followed by the payload, then flushes.</summary>
    public static void Write(Stream stream, ReadOnlySpan<byte> payload)
    {
        Span<byte> len = stackalloc byte[4];
        BinaryPrimitives.WriteInt32LittleEndian(len, payload.Length);
        stream.Write(len);
        stream.Write(payload);
        stream.Flush();
    }

    /// <summary>Reads one frame, or returns null at clean end-of-stream.</summary>
    public static byte[]? Read(Stream stream)
    {
        Span<byte> len = stackalloc byte[4];
        if (!TryReadExact(stream, len)) return null;
        int length = BinaryPrimitives.ReadInt32LittleEndian(len);
        if (length < 0) throw new InvalidDataException($"Negative frame length {length}.");
        var payload = new byte[length];
        if (length > 0 && !TryReadExact(stream, payload))
            throw new EndOfStreamException("Truncated frame payload.");
        return payload;
    }

    /// <summary>Fills <paramref name="buffer"/> across however many partial reads the stream returns. Returns
    /// false on a clean EOF before any byte (no frame pending); throws on EOF mid-buffer (a truncated frame).</summary>
    private static bool TryReadExact(Stream stream, Span<byte> buffer)
    {
        int offset = 0;
        while (offset < buffer.Length)
        {
            int read = stream.Read(buffer[offset..]);
            if (read == 0) return offset == 0 ? false : throw new EndOfStreamException();
            offset += read;
        }
        return true;
    }
}

/// <summary>Typed read/write helpers shared by both ends of the pipe (built on BinaryReader/Writer).</summary>
public static class Wire
{
    public static void WriteStringArray(BinaryWriter w, IReadOnlyList<string> items)
    {
        w.Write(items.Count);
        foreach (string s in items) w.Write(s);
    }

    public static string[] ReadStringArray(BinaryReader r)
    {
        int n = r.ReadInt32();
        var items = new string[n];
        for (int i = 0; i < n; i++) items[i] = r.ReadString();
        return items;
    }

    public static void WriteFloatArray(BinaryWriter w, ReadOnlySpan<float> data)
    {
        w.Write(data.Length);
        foreach (float f in data) w.Write(f);
    }

    public static float[] ReadFloatArray(BinaryReader r)
    {
        int n = r.ReadInt32();
        var data = new float[n];
        for (int i = 0; i < n; i++) data[i] = r.ReadSingle();
        return data;
    }

    public static void WriteDoubleArray(BinaryWriter w, ReadOnlySpan<double> data)
    {
        w.Write(data.Length);
        foreach (double d in data) w.Write(d);
    }

    public static double[] ReadDoubleArray(BinaryReader r)
    {
        int n = r.ReadInt32();
        var data = new double[n];
        for (int i = 0; i < n; i++) data[i] = r.ReadDouble();
        return data;
    }

    public static void WriteLongArray(BinaryWriter w, IReadOnlyList<long> data)
    {
        w.Write(data.Count);
        foreach (long v in data) w.Write(v);
    }

    public static long[] ReadLongArray(BinaryReader r)
    {
        int n = r.ReadInt32();
        var data = new long[n];
        for (int i = 0; i < n; i++) data[i] = r.ReadInt64();
        return data;
    }

    public static void WriteBytes(BinaryWriter w, byte[] data)
    {
        w.Write(data.Length);
        w.Write(data);
    }

    public static byte[] ReadBytes(BinaryReader r)
    {
        int n = r.ReadInt32();
        return r.ReadBytes(n);
    }
}
