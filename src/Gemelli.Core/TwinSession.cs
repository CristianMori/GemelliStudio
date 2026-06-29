using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Core;

/// <summary>
/// The two-process digital-twin orchestrator. Launches a physics worker (ovphysx) and a render
/// worker (ovrtx) as separate processes — so their conflicting Omniverse plugin sets never share a
/// process — and drives the loop each frame:
/// <c>physics.StepAndReadPoses → convert poses to USD matrices → render.WriteAndStep → frames</c>.
/// Implements <see cref="ISimApi"/> so controllers can read/write state. Not thread-safe; drive from one thread.
/// </summary>
public sealed class TwinSession : IDisposable, ISimApi, ITwinDriver
{
    private readonly SimulationOptions _options;
    private readonly string[] _renderProducts;

    private readonly WorkerHost _physics;
    private readonly WorkerHost _render;
    private readonly FrameBuffer? _frameBuffer; // shared-memory framebuffer (Windows); null = inline pipe transport

    private string[] _primPaths = [];
    private float _simTime;
    private bool _disposed;
    private IReadOnlyList<CapturedFrame> _latestFrames = [];

    private readonly object _camLock = new();
    private string? _cameraPath;
    private double[]? _cameraMatrix;

    /// <summary>
    /// Sets an interactive camera transform (USD row-vector 4×4, 16 doubles) written to
    /// <paramref name="primPath"/>'s <c>omni:xform</c> each render — lets the viewport orbit/pan/zoom
    /// live. Thread-safe; call from any thread.
    /// </summary>
    public void SetCameraTransform(string primPath, double[] matrix16)
    {
        if (matrix16.Length != 16) throw new ArgumentException("Camera matrix must be 16 doubles.", nameof(matrix16));
        lock (_camLock) { _cameraPath = primPath; _cameraMatrix = matrix16; }
    }

    public double SimTime => _simTime;
    public long FrameCount { get; private set; }

    /// <summary>
    /// Render the secondary products (everything after the first — typically sensor cameras) only every
    /// Nth frame, so an expensive depth/segmentation sensor doesn't halve the interactive viewport's
    /// frame rate. 1 = render every product every frame (default; required for headless dataset capture).
    /// </summary>
    public int SecondaryRenderInterval { get; set; } = 1;
    public int RigidBodyCount => _primPaths.Length;
    public (uint Major, uint Minor, uint Patch) RenderVersion { get; private set; }

    // ----- ITwinDriver -----
    ISimApi ITwinDriver.Api => this;

    // ----- ISimApi -----
    public IReadOnlyList<string> RigidBodyPaths => _primPaths;
    public IReadOnlyList<CapturedFrame> LatestFrames => _latestFrames;
    public CapturedFrame? Frame(string renderProduct) =>
        _latestFrames.FirstOrDefault(f => f.RenderProduct == renderProduct);

    public float[] Read(SimTensor channel, string pattern) => ReadShaped(channel, pattern).Data;

    public (long[] Shape, float[] Data) ReadShaped(SimTensor channel, string pattern)
    {
        EnsureNotDisposed();
        using BinaryReader r = _physics.Request((ushort)PhysicsOp.ReadTensor, w =>
        {
            w.Write((int)channel);
            w.Write(pattern);
        });
        long[] shape = Wire.ReadLongArray(r);
        float[] data = Wire.ReadFloatArray(r);
        return (shape, data);
    }

    public IReadOnlyList<string> DofNames(string pattern)
    {
        EnsureNotDisposed();
        using BinaryReader r = _physics.Request((ushort)PhysicsOp.ReadDofNames, w => w.Write(pattern));
        return Wire.ReadStringArray(r);
    }

    public void Write(SimTensor channel, string pattern, float[] values)
    {
        EnsureNotDisposed();
        _physics.Send((ushort)PhysicsOp.WriteTensor, w =>
        {
            w.Write((int)channel);
            w.Write(pattern);
            Wire.WriteFloatArray(w, values);
        });
    }

    public void SetDofPositionTargets(string pattern, float[] values) =>
        Write(SimTensor.ArticulationDofPositionTarget, pattern, values);

    public void SetDofVelocityTargets(string pattern, float[] values) =>
        Write(SimTensor.ArticulationDofVelocityTarget, pattern, values);

    /// <summary>Launches both worker processes, creates the shared framebuffer (Windows), then initializes
    /// each worker and loads the same USD into both. Disposes everything and rethrows on any setup failure.</summary>
    public TwinSession(SimulationOptions options)
    {
        _options = options ?? throw new ArgumentNullException(nameof(options));
        if (options.RenderProducts.Count == 0)
            throw new ArgumentException("At least one render product is required.", nameof(options));
        _renderProducts = options.RenderProducts.ToArray();

        string physicsPipe = "ovgemelli-phys-" + Guid.NewGuid().ToString("N");
        string renderPipe = "ovgemelli-rend-" + Guid.NewGuid().ToString("N");

        var physicsEnv = new Dictionary<string, string>();
        if (!string.IsNullOrEmpty(options.OvPhysxLibrary))
            physicsEnv[GemelliEnvironment.OvPhysxLibEnvVar] = options.OvPhysxLibrary;

        // Shared-memory framebuffer for zero-(pipe-)copy frame transport (Windows only).
        string frameBufName = "";
        long frameBufCap = 0;
        if (OperatingSystem.IsWindows())
        {
            frameBufName = "gemelli-frame-" + Guid.NewGuid().ToString("N");
            frameBufCap = options.FrameBufferBytes;
            _frameBuffer = FrameBuffer.Create(frameBufName, frameBufCap);
        }

        _physics = WorkerHost.Launch("physics", TwinHostLocator.PhysicsHost(options.PhysicsHostPath), physicsPipe, physicsEnv);
        _render = WorkerHost.Launch("render", TwinHostLocator.RenderHost(options.RenderHostPath), renderPipe);

        try
        {
            // Physics: init device, load USD, bind the rigid-body poses we will mirror.
            _physics.Send((ushort)PhysicsOp.Init, w => w.Write((int)options.Device));
            _physics.Send((ushort)PhysicsOp.LoadUsd, w => w.Write(options.UsdPath));
            using (BinaryReader r = _physics.Request((ushort)PhysicsOp.BindPoses, w => w.Write(options.RigidBodyPattern)))
                _primPaths = Wire.ReadStringArray(r);

            // Render: init renderer (with the shared framebuffer name), load the same USD.
            using (BinaryReader r = _render.Request((ushort)RenderOp.Init, w =>
            {
                w.Write(options.OvrtxLibraryDirectory ?? "");
                w.Write(options.RendererSyncMode);
                w.Write(frameBufName);
                w.Write(frameBufCap);
                w.Write(options.RenderEnabled);
            }))
            {
                RenderVersion = (r.ReadUInt32(), r.ReadUInt32(), r.ReadUInt32());
            }
            _render.Send((ushort)RenderOp.LoadUsd, w => w.Write(options.UsdPath));
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    /// <summary>Renders warmup frames (render-only) to prime the RTX pipeline before the first real frame.</summary>
    public void Warmup()
    {
        EnsureNotDisposed();
        _render.Send((ushort)RenderOp.Warmup, w =>
        {
            Wire.WriteStringArray(w, _renderProducts);
            w.Write(_options.WarmupFrames);
        });
    }

    /// <summary>Wall-clock milliseconds spent in the physics round-trip on the last <see cref="Step"/>.</summary>
    public double LastPhysicsMs { get; private set; }

    /// <summary>Wall-clock milliseconds spent in the render round-trip on the last <see cref="Step"/>.</summary>
    public double LastRenderMs { get; private set; }

    /// <summary>Advances one frame across both processes and returns the captured sensor frames: steps
    /// physics by <paramref name="physicsSubsteps"/> fixed substeps, mirrors the final poses (plus any live
    /// camera) into the renderer, and renders in a single round-trip. Records per-phase timings.</summary>
    public IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1)
    {
        EnsureNotDisposed();
        if (physicsSubsteps < 1) physicsSubsteps = 1;

        // 1. Advance physics by N fixed substeps; keep only the final poses to mirror into the renderer.
        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        float[] poses = [];
        int n = 0;
        for (int i = 0; i < physicsSubsteps; i++)
        {
            using BinaryReader r = _physics.Request((ushort)PhysicsOp.StepAndReadPoses, w =>
            {
                w.Write(_options.TimeStep);
                w.Write(_simTime);
            });
            n = r.ReadInt32();
            poses = Wire.ReadFloatArray(r);
            _simTime += _options.TimeStep;
        }
        long t1 = System.Diagnostics.Stopwatch.GetTimestamp();

        // 2. Convert final poses -> USD 4×4 matrices (pure, in-process). Append the interactive
        //    camera transform (if the UI has set one) so the viewport can orbit/pan/zoom live.
        double[] bodyMatrices = n > 0 ? TransformConversion.PosesToUsdMatrices(poses) : [];
        string[] writePaths = _primPaths;
        double[] matrices = bodyMatrices;
        lock (_camLock)
        {
            if (_cameraPath is not null && _cameraMatrix is not null)
            {
                writePaths = [.. _primPaths, _cameraPath];
                matrices = [.. bodyMatrices, .. _cameraMatrix];
            }
        }

        // 3. Render worker: write transforms + step + return frames (single round-trip). Render the
        //    secondary (sensor) products only every Nth frame to keep the primary viewport responsive.
        string[] renderProducts = _renderProducts;
        if (_renderProducts.Length > 1 && SecondaryRenderInterval > 1 && FrameCount % SecondaryRenderInterval != 0)
            renderProducts = [_renderProducts[0]];

        List<CapturedFrame> frames;
        using (BinaryReader r = _render.Request((ushort)RenderOp.WriteAndStep, w =>
        {
            Wire.WriteStringArray(w, writePaths);
            Wire.WriteDoubleArray(w, matrices);
            Wire.WriteStringArray(w, renderProducts);
            w.Write((double)_options.TimeStep * physicsSubsteps);
        }))
        {
            bool sharedMem = r.ReadBoolean();
            frames = sharedMem && _frameBuffer is not null
                ? FrameLayoutCodec.ReadAndMaterialize(r, _frameBuffer) // pixels from shared memory
                : FrameCodec.Read(r);                                  // pixels inline (fallback)
        }
        long t2 = System.Diagnostics.Stopwatch.GetTimestamp();

        double toMs = 1000.0 / System.Diagnostics.Stopwatch.Frequency;
        LastPhysicsMs = (t1 - t0) * toMs;
        LastRenderMs = (t2 - t1) * toMs;

        FrameCount++;
        _latestFrames = frames;
        return frames;
    }

    private void EnsureNotDisposed()
    {
        if (_disposed) throw new ObjectDisposedException(nameof(TwinSession));
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        // Ask each worker to shut down cleanly; ignore failures (it may already be dead).
        if (_physics is not null && _physics.IsAlive)
            try { _physics.Send((ushort)PhysicsOp.Shutdown); } catch { /* already gone */ }
        if (_render is not null && _render.IsAlive)
            try { _render.Send((ushort)RenderOp.Shutdown); } catch { /* already gone */ }

        _physics?.Dispose();
        _render?.Dispose();
        _frameBuffer?.Dispose();
    }
}
