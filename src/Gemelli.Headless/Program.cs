using Gemelli.Core;
using Gemelli.Core.Control;
using Gemelli.Core.Imaging;
using Gemelli.Core.Sensors;
using Gemelli.Scripting;

// Gemelli headless host: drive the two-process twin (ovphysx + ovrtx workers) from C#,
// and record each camera's color output as PNG.
//
// Usage:
//   Gemelli.Headless --usd <path> --products <p1,p2,...> [options]
// Options:
//   --steps <n>            frames to simulate (default 60)
//   --out <dir>           output directory for PNGs (default ./out)
//   --dt <seconds>        timestep (default 0.016667)
//   --rigid <glob>        rigid-body pattern bridged to the renderer (default /World/*)
//   --ovrtx-lib <dir>     directory containing ovrtx-dynamic.dll
//   --ovphysx-lib <path>  absolute path to ovphysx.dll (else read from OVPHYSX_LIB)
//   --device cpu|gpu|auto ovphysx device (default auto)
//   --script <path.csx>   per-frame C# behavior script (hot-reloaded); else passive playback

HeadlessOptions? options = ArgParser.Parse(args);
if (options is null)
{
    ArgParser.PrintUsage();
    return 1;
}

Directory.CreateDirectory(options.OutputDir);

try
{
    using var twin = new TwinSession(options.Sim);
    Console.WriteLine($"Twin up. ovrtx {twin.RenderVersion}. Rigid bodies bridged: {twin.RigidBodyCount}. " +
                      $"Render products: {string.Join(", ", options.Sim.RenderProducts)}");

    var runner = new TwinRunner(twin);
    if (!string.IsNullOrEmpty(options.ScriptPath))
    {
        runner.Add(new ScriptController(options.ScriptPath));
        Console.WriteLine($"Controller: script '{options.ScriptPath}'");
    }
    else
    {
        runner.Add(new PlaybackController());
        Console.WriteLine("Controller: passive playback");
    }

    Console.WriteLine($"Warming up ({options.Sim.WarmupFrames} frames; first run compiles shaders)...");
    runner.Start();

    // Optional synthetic-data recorder (color + depth + segmentation + manifest) per render product.
    SensorRecorder? recorder = options.RecordDir is null ? null : new SensorRecorder(options.RecordDir);
    if (recorder is not null) Console.WriteLine($"Recording dataset → {Path.GetFullPath(options.RecordDir!)}");

    double physTotal = 0, renderTotal = 0;
    int measured = 0;
    for (int step = 0; step < options.Steps; step++)
    {
        IReadOnlyList<CapturedFrame> frames = runner.Step();
        foreach (CapturedFrame frame in frames)
        {
            SaveColor(frame, step, options.OutputDir);
            SaveDepth(frame, step, options.OutputDir);
            recorder?.Submit(step, frame);
            if (step == options.Steps - 1)
                foreach (var (name, v) in frame.Vars)
                    Console.WriteLine($"  var '{name}'  shape=[{string.Join("x", v.Shape)}]  {v.ElementType}{v.ElementBits}x{v.Lanes}  {v.Bytes.Length} bytes");
        }

        // Skip the first few frames (pipeline ramp-up) when averaging.
        if (step >= 3) { physTotal += twin.LastPhysicsMs; renderTotal += twin.LastRenderMs; measured++; }

        if (step % 10 == 0 || step == options.Steps - 1)
            Console.WriteLine($"  step {step + 1}/{options.Steps}  t={twin.SimTime:F3}s  physics {twin.LastPhysicsMs:F1}ms  render {twin.LastRenderMs:F1}ms");
    }
    if (options.DumpDof is { } robot)
    {
        IReadOnlyList<string> names = twin.DofNames(robot);
        float[] dof = twin.Read(Gemelli.Core.Ipc.SimTensor.ArticulationDofPosition, robot);
        Console.WriteLine($"DOF dump for '{robot}' after {options.Steps} steps: {names.Count} names");
        for (int i = 0; i < Math.Min(names.Count, dof.Length); i++)
            Console.WriteLine($"  {names[i]} = {dof[i]:F4}");
    }

    runner.Stop();
    if (recorder is not null)
    {
        recorder.Dispose(); // flush the writer thread
        Console.WriteLine($"Recorded {recorder.Written} frames ({recorder.Dropped} dropped) to '{Path.GetFullPath(options.RecordDir!)}'.");
    }

    if (measured > 0)
    {
        double pAvg = physTotal / measured, rAvg = renderTotal / measured;
        Console.WriteLine($"AVERAGE over {measured} frames: physics {pAvg:F1}ms  render {rAvg:F1}ms  total {pAvg + rAvg:F1}ms  (~{1000.0 / (pAvg + rAvg):F0} fps)");
    }
    Console.WriteLine($"Done. {twin.FrameCount} frames simulated; PNGs in '{Path.GetFullPath(options.OutputDir)}'.");
    return 0;
}
catch (Exception ex)
{
    Console.Error.WriteLine($"[error] {ex.Message}");
    return 2;
}

// Encode the frame's color render var to PNG (3- or 4-channel only) named <product>_<step>.png.
static void SaveColor(CapturedFrame frame, int step, string outDir)
{
    RenderVarData? color = frame.Color;
    if (color is null || color.Width == 0 || color.Height == 0)
        return;
    int channels = color.Channels;
    if (channels is not (3 or 4))
        return;

    byte[] png = Png.Encode(color.Bytes, color.Width, color.Height, channels);
    string safeProduct = frame.RenderProduct.Trim('/').Replace('/', '_');
    File.WriteAllBytes(Path.Combine(outDir, $"{safeProduct}_{step:D4}.png"), png);
}

// Save a normalized-grayscale depth preview (near = bright) when the product carries depth.
static void SaveDepth(CapturedFrame frame, int step, string outDir)
{
    if (SensorVisualize.DepthGray(frame.Depth) is not { } d) return;
    byte[] png = Png.Encode(d.Item3, d.Item1, d.Item2, 4);
    string safeProduct = frame.RenderProduct.Trim('/').Replace('/', '_');
    File.WriteAllBytes(Path.Combine(outDir, $"{safeProduct}_{step:D4}_depth.png"), png);
}

internal sealed record HeadlessOptions(SimulationOptions Sim, int Steps, string OutputDir, string? ScriptPath, string? RecordDir, string? DumpDof);

/// <summary>Parses the headless CLI argv into <see cref="HeadlessOptions"/>; returns null on missing/malformed required args.</summary>
internal static class ArgParser
{
    /// <summary>Splits argv into presence flags and <c>--key value</c> pairs, then builds the typed options.</summary>
    public static HeadlessOptions? Parse(string[] args)
    {
        // Presence flags (no value) — pull them out before the key/value pairing below.
        bool noRender = args.Contains("--no-render", StringComparer.OrdinalIgnoreCase);
        if (noRender) args = args.Where(a => !string.Equals(a, "--no-render", StringComparison.OrdinalIgnoreCase)).ToArray();

        var map = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        for (int i = 0; i < args.Length - 1; i += 2)
        {
            if (!args[i].StartsWith("--", StringComparison.Ordinal)) return null;
            map[args[i][2..]] = args[i + 1];
        }

        if (!map.TryGetValue("usd", out string? usd) || string.IsNullOrWhiteSpace(usd)) return null;
        if (!map.TryGetValue("products", out string? products) || string.IsNullOrWhiteSpace(products)) return null;

        PhysicsDevice device = map.TryGetValue("device", out string? d)
            ? d.ToLowerInvariant() switch { "cpu" => PhysicsDevice.Cpu, "gpu" => PhysicsDevice.Gpu, _ => PhysicsDevice.Auto }
            : PhysicsDevice.Auto;

        var sim = new SimulationOptions
        {
            UsdPath = usd,
            RenderProducts = products.Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries),
            Device = device,
            TimeStep = map.TryGetValue("dt", out string? dt) ? float.Parse(dt) : 1f / 60f,
            RigidBodyPattern = map.GetValueOrDefault("rigid", "/World/*"),
            OvrtxLibraryDirectory = map.GetValueOrDefault("ovrtx-lib") ?? GemelliEnvironment.ResolveOvrtxDirectory(),
            OvPhysxLibrary = map.GetValueOrDefault("ovphysx-lib")
                             ?? GemelliEnvironment.ResolveOvPhysxLibrary(),
            RenderEnabled = !noRender,
        };

        int steps = map.TryGetValue("steps", out string? s) ? int.Parse(s) : 60;
        string outDir = map.GetValueOrDefault("out", "out");
        return new HeadlessOptions(sim, steps, outDir, map.GetValueOrDefault("script"), map.GetValueOrDefault("record"), map.GetValueOrDefault("dump-dof"));
    }

    public static void PrintUsage() => Console.Error.WriteLine(
        "Usage: Gemelli.Headless --usd <path> --products <p1,p2,...> " +
        "[--steps n] [--out dir] [--record dir] [--dt sec] [--rigid glob] [--ovrtx-lib dir] [--ovphysx-lib path] [--device cpu|gpu|auto] [--script path.csx]");
}
