using Gemelli.Core;
using Gemelli.Core.Sensors;
using Xunit;
using Xunit.Abstractions;

namespace Gemelli.Tests;

/// <summary>
/// Tier-2 end-to-end test of the two-process twin: ovphysx physics worker + ovrtx render worker
/// orchestrated by <see cref="TwinSession"/>. Gated on <c>OVPHYSX_LIB</c> and <c>GEMELLI_OVRTX_DIR</c>;
/// skips cleanly otherwise. Requires the worker executables to be built and an RTX GPU.
/// </summary>
public class TwinLiveTests
{
    private readonly ITestOutputHelper _out;

    private static string? OvrtxDir => Environment.GetEnvironmentVariable("GEMELLI_OVRTX_DIR");
    private static string? OvPhysxLib => Environment.GetEnvironmentVariable(GemelliEnvironment.OvPhysxLibEnvVar);
    private static bool Enabled => !string.IsNullOrEmpty(OvPhysxLib) && !string.IsNullOrEmpty(OvrtxDir);

    private static string SampleUsd =>
        Environment.GetEnvironmentVariable("GEMELLI_SAMPLE_USD")
        ?? @"C:\DataDrive\ovGemelli\native\ovphysx\ovphysx\samples\data\boxes_falling_on_groundplane.usda";

    private const string RenderProduct =
        "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0";

    public TwinLiveTests(ITestOutputHelper output) => _out = output;

    // End-to-end: bridged rigid bodies are detected and the twin loop produces a non-black color frame.
    [Fact]
    public void Twin_Loop_Renders_Falling_Boxes()
    {
        if (!Enabled) return;

        var options = new SimulationOptions
        {
            UsdPath = SampleUsd,
            RenderProducts = [RenderProduct],
            Device = PhysicsDevice.Cpu,        // ovphysx worker on CPU; ovrtx worker uses the GPU
            RigidBodyPattern = "/World/Cube*",
            OvrtxLibraryDirectory = OvrtxDir,
            OvPhysxLibrary = OvPhysxLib,
            WarmupFrames = 8,
        };

        using var twin = new TwinSession(options);
        _out.WriteLine($"ovrtx {twin.RenderVersion}; rigid bodies bridged: {twin.RigidBodyCount}");
        Assert.True(twin.RigidBodyCount > 0);

        twin.Warmup();

        CapturedFrame? colorFrame = null;
        for (int step = 0; step < 20; step++)
        {
            IReadOnlyList<CapturedFrame> frames = twin.Step();
            colorFrame = frames.FirstOrDefault(f => f.Color is not null) ?? colorFrame;
        }

        Assert.NotNull(colorFrame);
        RenderVarData color = colorFrame!.Color!;
        _out.WriteLine($"color frame: {color.Width}x{color.Height}x{color.Channels}, {color.Bytes.Length} bytes");

        Assert.True(color.Width > 0 && color.Height > 0, "rendered frame has zero dimensions");
        Assert.True(color.Bytes.Any(b => b != 0), "rendered color buffer was entirely zero");
    }
}
