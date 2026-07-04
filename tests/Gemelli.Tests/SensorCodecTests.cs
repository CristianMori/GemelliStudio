using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for the sensor frame transport: the inline pipe codec (<see cref="FrameCodec"/>),
/// the shared-memory layout codec (<see cref="FrameLayoutCodec"/> + <see cref="FrameBuffer"/>), and
/// the buffer's bounds checks. Both processes must agree on these byte-for-byte; a single field-order
/// slip turns every sensor frame into garbage.
/// </summary>
public class SensorCodecTests
{
    private static CapturedFrame SampleFrame(string product = "/Render/Cam") => new(
        product, StartTime: 1.25, EndTime: 1.5,
        new Dictionary<string, RenderVarData>
        {
            ["LdrColor"] = new("LdrColor", [2, 3], ScalarType.UInt, 8, 4, [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
                                                                           13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24]),
            // Depth's real-world quirk: float32 bits tagged as UInt32, 1 lane.
            ["DistanceToImagePlaneSD"] = new("DistanceToImagePlaneSD", [2, 3], ScalarType.UInt, 32, 1,
                BitConverter.GetBytes(1.5f).Concat(BitConverter.GetBytes(float.PositiveInfinity))
                    .Concat(new byte[4 * 4]).ToArray()),
            ["Empty"] = new("Empty", [0], ScalarType.Float, 32, 1, []),
        });

    private static void AssertFramesEqual(CapturedFrame expected, CapturedFrame actual)
    {
        Assert.Equal(expected.RenderProduct, actual.RenderProduct);
        Assert.Equal(expected.StartTime, actual.StartTime);
        Assert.Equal(expected.EndTime, actual.EndTime);
        Assert.Equal(expected.Vars.Keys.OrderBy(k => k), actual.Vars.Keys.OrderBy(k => k));
        foreach (var (name, v) in expected.Vars)
        {
            RenderVarData a = actual.Vars[name];
            Assert.Equal(v.Name, a.Name);
            Assert.Equal(v.Shape, a.Shape);
            Assert.Equal(v.ElementType, a.ElementType);
            Assert.Equal(v.ElementBits, a.ElementBits);
            Assert.Equal(v.Lanes, a.Lanes);
            Assert.Equal(v.Bytes, a.Bytes);
        }
    }

    // ---- inline pipe path ----

    [Fact]
    public void FrameCodec_Round_Trips_Frames_And_Vars()
    {
        List<CapturedFrame> frames = [SampleFrame("/Render/CamA"), SampleFrame("/Render/CamB")];

        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
            FrameCodec.Write(w, frames);

        ms.Position = 0;
        using var r = new BinaryReader(ms);
        List<CapturedFrame> read = FrameCodec.Read(r);

        Assert.Equal(2, read.Count);
        AssertFramesEqual(frames[0], read[0]);
        AssertFramesEqual(frames[1], read[1]);
        Assert.Equal(ms.Length, ms.Position); // consumed exactly what was written
    }

    [Fact]
    public void FrameCodec_Round_Trips_An_Empty_Frame_List()
    {
        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
            FrameCodec.Write(w, []);
        ms.Position = 0;
        Assert.Empty(FrameCodec.Read(new BinaryReader(ms)));
    }

    // ---- shared-memory path (Windows named mapping; both sides in-process here) ----

    [Fact]
    public void FrameLayoutCodec_Round_Trips_Pixels_Through_A_Shared_Buffer()
    {
        if (!OperatingSystem.IsWindows()) return; // named mappings are Windows-only, like the runtime path

        CapturedFrame frame = SampleFrame();
        using var buffer = FrameBuffer.Create("gemelli-test-" + Guid.NewGuid().ToString("N"), 4096);

        // Worker side: pixels into the buffer, layout over the "pipe".
        long offset = 0;
        var vars = new List<VarLayout>();
        foreach (var (name, v) in frame.Vars)
        {
            buffer.Write(offset, v.Bytes);
            vars.Add(new VarLayout(name, v.Shape, v.ElementType, v.ElementBits, v.Lanes, offset, v.Bytes.Length));
            offset += v.Bytes.Length;
        }
        List<FrameLayout> layout = [new(frame.RenderProduct, frame.StartTime, frame.EndTime, vars)];

        using var ms = new MemoryStream();
        using (var w = new BinaryWriter(ms, System.Text.Encoding.UTF8, leaveOpen: true))
            FrameLayoutCodec.Write(w, layout);

        // Host side: read the layout and materialize pixels back out of the shared buffer.
        ms.Position = 0;
        List<CapturedFrame> read = FrameLayoutCodec.ReadAndMaterialize(new BinaryReader(ms), buffer);

        Assert.Single(read);
        AssertFramesEqual(frame, read[0]);
    }

    [Fact]
    public void FrameBuffer_Rejects_Out_Of_Range_And_Overflowing_Access()
    {
        if (!OperatingSystem.IsWindows()) return;

        using var buffer = FrameBuffer.Create("gemelli-test-" + Guid.NewGuid().ToString("N"), 64);

        buffer.Write(60, new byte[4]);                                                    // exactly at the end: fine
        Assert.Equal(new byte[4], buffer.Read(60, 4));

        Assert.Throws<ArgumentOutOfRangeException>(() => buffer.Write(-1, new byte[1]));  // negative offset
        Assert.Throws<ArgumentOutOfRangeException>(() => buffer.Write(61, new byte[4]));  // past the end
        Assert.Throws<ArgumentOutOfRangeException>(() => buffer.Read(0, 65));             // longer than capacity
        Assert.Throws<ArgumentOutOfRangeException>(() => buffer.Read(0, -1));             // negative length
        // A corrupt near-MaxValue offset must not wrap the bounds check via signed overflow.
        Assert.Throws<ArgumentOutOfRangeException>(() => buffer.Read(long.MaxValue - 2, 8));
    }
}
