namespace Gemelli.Core;

/// <summary>Physics device selection (maps to ovphysx <c>DeviceType</c> inside the physics worker).</summary>
public enum PhysicsDevice
{
    Auto = 0,
    Gpu = 1,
    Cpu = 2,
}

/// <summary>Configuration for a <see cref="TwinSession"/> (the two-process orchestrator).</summary>
public sealed record SimulationOptions
{
    /// <summary>Path (or URL) to the Isaac Sim–exported USD loaded into both workers. Required.</summary>
    public required string UsdPath { get; init; }

    /// <summary>Render product prim paths to render each step (cameras / lidar / radar). At least one required.</summary>
    public required IReadOnlyList<string> RenderProducts { get; init; }

    /// <summary>ovphysx simulation device.</summary>
    public PhysicsDevice Device { get; init; } = PhysicsDevice.Auto;

    /// <summary>Fixed simulation timestep, seconds. Live-adjustable via <see cref="Control.TwinService.TimeStep"/>.</summary>
    public float TimeStep { get; init; } = 1f / 60f;

    /// <summary>
    /// Sim-time-to-wall-clock factor for real-time playback: 1 = real speed, 10 = 10× accelerated, 0.5 =
    /// half-speed slow-mo. The achievable acceleration is bounded by physics throughput (sim time advanced
    /// per wall-second ≈ <see cref="TimeStep"/> / physics-step-ms); asking beyond that just runs flat-out.
    /// Only meaningful when <see cref="RealTimePlayback"/> is true. Live-adjustable via
    /// <see cref="Control.TwinService.TimeScale"/>.
    /// </summary>
    public float TimeScale { get; init; } = 1f;

    /// <summary>
    /// ovphysx glob pattern selecting the rigid bodies whose world pose is mirrored into the renderer
    /// each frame. Defaults to the <b>recursive</b> <c>"/World/**"</c> so nested bodies — including
    /// articulation links (e.g. a robot arm's <c>panda_link1..7</c>) which ovphysx also exposes as
    /// rigid bodies — are bridged, not just direct children of <c>/World</c>. Empty/unmatched is allowed.
    /// </summary>
    public string RigidBodyPattern { get; init; } = "/World/**";

    /// <summary>Directory containing <c>ovrtx-dynamic.dll</c> and its runtime dirs (passed to the render worker).</summary>
    public string? OvrtxLibraryDirectory { get; init; }

    /// <summary>Absolute path to <c>ovphysx.dll</c> (passed to the physics worker via <c>OVPHYSX_LIB</c>).</summary>
    public string? OvPhysxLibrary { get; init; }

    /// <summary>Run the renderer in synchronous mode (simpler stepping; good for headless/batch).</summary>
    public bool RendererSyncMode { get; init; } = true;

    /// <summary>
    /// When false, the render worker runs the full per-frame pipeline (receive poses, write transforms,
    /// round-trip) but skips the expensive ovrtx render step and returns empty frames. Used to measure
    /// the simulation/bridge cost in isolation from RTX rendering.
    /// </summary>
    public bool RenderEnabled { get; init; } = true;

    /// <summary>Number of render-only warmup steps before the first real frame (primes the RTX pipeline).</summary>
    public int WarmupFrames { get; init; } = 8;

    /// <summary>
    /// When true, continuous play advances physics to match wall-clock time (real-time speed) by
    /// running extra fixed substeps per render when the render is slower than the timestep. When false,
    /// play runs as fast as possible (one substep per render — good for batch throughput).
    /// </summary>
    public bool RealTimePlayback { get; init; } = true;

    /// <summary>Upper bound on physics substeps per rendered frame (prevents a spiral-of-death after a hitch).</summary>
    public int MaxPhysicsSubstepsPerFrame { get; init; } = 6;

    /// <summary>
    /// Capacity (bytes) of the shared-memory framebuffer used to transport rendered frames from the
    /// render worker without serializing them through the pipe. Must hold one frame's render vars;
    /// default 64 MB covers 4K RGBA + depth. Windows only; ignored elsewhere.
    /// </summary>
    public long FrameBufferBytes { get; init; } = 64L * 1024 * 1024;

    /// <summary>Override paths to the worker executables (defaults resolve next to the orchestrator assembly).</summary>
    public string? PhysicsHostPath { get; init; }
    public string? RenderHostPath { get; init; }
}
