using Gemelli.Core.Ipc;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for the pipe framing (<see cref="Frame"/>) and the typed array codecs
/// (<see cref="Wire"/>) — the protocol layer both worker processes depend on. Pure streams, no IPC.
/// </summary>
public class WireTests
{
    // ---- Frame: [int32 length][payload] framing ----

    [Fact]
    public void Frame_Round_Trips_Multiple_Frames_On_One_Stream()
    {
        using var ms = new MemoryStream();
        byte[] a = [1, 2, 3, 4, 5];
        byte[] b = [255];
        Frame.Write(ms, a);
        Frame.Write(ms, b);

        ms.Position = 0;
        Assert.Equal(a, Frame.Read(ms));
        Assert.Equal(b, Frame.Read(ms));
        Assert.Null(Frame.Read(ms)); // clean EOF after the last frame
    }

    [Fact]
    public void Frame_Round_Trips_A_Zero_Length_Payload()
    {
        using var ms = new MemoryStream();
        Frame.Write(ms, ReadOnlySpan<byte>.Empty);

        ms.Position = 0;
        byte[]? payload = Frame.Read(ms);
        Assert.NotNull(payload);
        Assert.Empty(payload);
        Assert.Null(Frame.Read(ms));
    }

    [Fact]
    public void Frame_Read_Returns_Null_On_Clean_End_Of_Stream()
    {
        using var ms = new MemoryStream();
        Assert.Null(Frame.Read(ms));
    }

    [Fact]
    public void Frame_Read_Rejects_A_Negative_Length_Prefix()
    {
        using var ms = new MemoryStream([0xFF, 0xFF, 0xFF, 0xFF]); // -1 little-endian
        Assert.Throws<InvalidDataException>(() => Frame.Read(ms));
    }

    [Fact]
    public void Frame_Read_Throws_On_A_Truncated_Payload()
    {
        using var ms = new MemoryStream([10, 0, 0, 0, 1, 2, 3]); // declares 10 bytes, carries 3
        Assert.Throws<EndOfStreamException>(() => Frame.Read(ms));
    }

    [Fact]
    public void Frame_Read_Throws_On_A_Truncated_Length_Prefix()
    {
        using var ms = new MemoryStream([10, 0]); // EOF mid-prefix is a torn frame, not a clean EOF
        Assert.Throws<EndOfStreamException>(() => Frame.Read(ms));
    }

    // Pipes return partial reads; the framing must assemble a frame across many one-byte reads.
    [Fact]
    public void Frame_Read_Assembles_Across_Partial_Reads()
    {
        using var inner = new MemoryStream();
        byte[] payload = [9, 8, 7, 6, 5, 4, 3, 2, 1];
        Frame.Write(inner, payload);
        inner.Position = 0;

        Assert.Equal(payload, Frame.Read(new OneByteAtATimeStream(inner)));
    }

    /// <summary>Wraps a stream so every Read returns at most one byte, mimicking a dribbling pipe.</summary>
    private sealed class OneByteAtATimeStream(Stream inner) : Stream
    {
        public override int Read(byte[] buffer, int offset, int count) =>
            inner.Read(buffer, offset, Math.Min(1, count));
        public override int Read(Span<byte> buffer) => inner.Read(buffer[..Math.Min(1, buffer.Length)]);

        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => false;
        public override long Length => inner.Length;
        public override long Position { get => inner.Position; set => throw new NotSupportedException(); }
        public override void Flush() { }
        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
    }

    // ---- Wire: typed array codecs ----

    [Fact]
    public void Wire_Array_Codecs_Round_Trip()
    {
        using var ms = new MemoryStream();
        string[] strings = ["", "/World/robot", "héllo ✓"];
        float[] floats = [0f, -1.5f, float.MaxValue, float.NegativeInfinity, float.NaN];
        double[] doubles = [0.0, Math.PI, double.Epsilon];
        long[] longs = [0, -1, long.MaxValue];
        byte[] bytes = [0, 127, 255];

        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
        {
            Wire.WriteStringArray(w, strings);
            Wire.WriteFloatArray(w, floats);
            Wire.WriteDoubleArray(w, doubles);
            Wire.WriteLongArray(w, longs);
            Wire.WriteBytes(w, bytes);
        }

        ms.Position = 0;
        using var r = new BinaryReader(ms);
        Assert.Equal(strings, Wire.ReadStringArray(r));
        Assert.Equal(floats, Wire.ReadFloatArray(r));
        Assert.Equal(doubles, Wire.ReadDoubleArray(r));
        Assert.Equal(longs, Wire.ReadLongArray(r));
        Assert.Equal(bytes, Wire.ReadBytes(r));
        Assert.Equal(ms.Length, ms.Position); // codecs consumed exactly what they wrote
    }

    [Fact]
    public void Wire_Empty_Arrays_Round_Trip()
    {
        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
        {
            Wire.WriteStringArray(w, []);
            Wire.WriteFloatArray(w, ReadOnlySpan<float>.Empty);
            Wire.WriteBytes(w, []);
        }

        ms.Position = 0;
        using var r = new BinaryReader(ms);
        Assert.Empty(Wire.ReadStringArray(r));
        Assert.Empty(Wire.ReadFloatArray(r));
        Assert.Empty(Wire.ReadBytes(r));
    }
}
