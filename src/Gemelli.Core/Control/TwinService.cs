using System.Collections.Concurrent;
using System.Diagnostics;
using Gemelli.Core.Imaging;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Core.Control;

/// <summary>
/// Thread-safe, sim-thread-owned wrapper around a <see cref="TwinSession"/> + <see cref="TwinRunner"/>.
/// All native/IPC access happens on a single dedicated thread (the twin and its pipes are not
/// thread-safe); callers from any thread post work via <see cref="Invoke{T}"/> or the lifecycle
/// methods. Supports both continuous play (for the UI) and explicit stepping (for an agent/MCP).
/// This is the shared core consumed by both the MCP server and the Avalonia Studio UI.
/// </summary>
public sealed class TwinService : IDisposable
{
    public enum RunState { Stopped, Paused, Playing, Faulted }

    private readonly BlockingCollection<Action> _queue = new();
    private readonly Func<SimulationOptions, ITwinDriver> _driverFactory;
    private Thread? _thread;
    private ITwinDriver? _driver;
    private TwinRunner? _runner;
    private volatile RunState _state = RunState.Stopped;
    private volatile IReadOnlyList<CapturedFrame> _latest = [];
    private volatile IReadOnlyDictionary<string, float[]> _poseCache = new Dictionary<string, float[]>();
    private volatile bool _stopRequested;
    private Exception? _fault;

    // Live-adjustable pacing (read each loop iteration). Seeded from SimulationOptions at Start; the UI can
    // change them while the twin runs. float is volatile-legal (double is not), so these stay lock-free.
    private volatile float _timeScale = 1f;
    private volatile float _timeStep = 1f / 60f;

    /// <summary>
    /// Sim-time-to-wall-clock factor (1 = real speed, 10 = 10× accelerated, &lt;1 = slow-mo). Applied to
    /// real-time playback; the actual speed-up is capped by how fast physics can step. Thread-safe; takes
    /// effect on the next loop iteration.
    /// </summary>
    public float TimeScale
    {
        get => _timeScale;
        set => _timeScale = Math.Clamp(value, 0.01f, 100f);
    }

    /// <summary>Fixed physics timestep in seconds. Thread-safe; takes effect on the next loop iteration.</summary>
    public float TimeStep
    {
        get => _timeStep;
        set => _timeStep = Math.Max(1e-4f, value);
    }

    /// <summary>Raised on the sim thread after each step with the captured frames.</summary>
    public event Action<IReadOnlyList<CapturedFrame>>? FrameProduced;

    /// <summary>
    /// Raised (off the UI thread) when the run loop dies from an unexpected error — typically a worker
    /// crash mid-run. The twin is torn down to <see cref="RunState.Faulted"/>; the host stays alive.
    /// </summary>
    public event Action<Exception>? Faulted;

    /// <summary>The exception that faulted the twin, if any.</summary>
    public Exception? FaultException => _fault;

    /// <param name="driverFactory">
    /// Builds the twin driver from options. Defaults to launching a real two-process
    /// <see cref="TwinSession"/>; tests inject a fake to exercise the threading without native libs.
    /// </param>
    public TwinService(Func<SimulationOptions, ITwinDriver>? driverFactory = null) =>
        _driverFactory = driverFactory ?? (o => new TwinSession(o));

    public RunState State => _state;
    public bool IsRunning => _state is RunState.Paused or RunState.Playing;
    public IReadOnlyList<CapturedFrame> LatestFrames => _latest;

    public double SimTime => _driver?.Api.SimTime ?? 0;
    public long FrameCount => _driver?.Api.FrameCount ?? 0;
    public IReadOnlyList<string> RigidBodyPaths => _driver?.Api.RigidBodyPaths ?? [];

    /// <summary>
    /// Latest cached world pose (px,py,pz,qx,qy,qz,qw) for a rigid body, or null. Lock-free and
    /// non-blocking — safe to call at frame rate from the UI thread (unlike <see cref="Invoke{T}"/>).
    /// Updated once per simulated frame on the sim thread.
    /// </summary>
    public float[]? TryGetPose(string path) => _poseCache.TryGetValue(path, out float[]? p) ? p : null;
    public (uint Major, uint Minor, uint Patch) RenderVersion =>
        _driver is TwinSession ts ? ts.RenderVersion : default;

    /// <summary>Last step's physics / render round-trip times (ms), when backed by a real twin.</summary>
    public double LastPhysicsMs => _driver is TwinSession s ? s.LastPhysicsMs : 0;
    public double LastRenderMs => _driver is TwinSession s ? s.LastRenderMs : 0;

    /// <summary>Sets the interactive viewport camera transform (USD 4×4, 16 doubles). Thread-safe.</summary>
    public void SetCameraTransform(string primPath, double[] matrix16) =>
        (_driver as TwinSession)?.SetCameraTransform(primPath, matrix16);

    /// <summary>Render secondary (sensor) products only every Nth frame; keeps the viewport responsive.</summary>
    public void SetSecondaryRenderInterval(int n)
    {
        if (_driver is TwinSession ts) ts.SecondaryRenderInterval = Math.Max(1, n);
    }

    /// <summary>
    /// Launches the twin on a dedicated sim thread (workers start, USD loads, warmup runs), then
    /// enters the run loop in <see cref="RunState.Paused"/>. Blocks until startup completes or fails.
    /// </summary>
    public void Start(SimulationOptions options, IEnumerable<IController>? controllers = null)
    {
        if (IsRunning) throw new InvalidOperationException("Twin is already running.");
        // Allow restart after Stop or Faulted.
        _thread?.Join(5_000);
        _stopRequested = false;
        _fault = null;
        _state = RunState.Stopped;

        var ready = new TaskCompletionSource();
        IController[] ctrls = controllers?.ToArray() ?? [];

        _thread = new Thread(() => RunLoop(options, ctrls, ready)) { IsBackground = true, Name = "ovGemelli-sim" };
        _thread.Start();
        ready.Task.GetAwaiter().GetResult(); // surface startup exceptions to the caller
    }

    // The sim thread's body: builds the driver/runner (signalling startup via <paramref name="ready"/>), then
    // loops draining queued commands and stepping while Playing, and finally tears the twin down on exit/fault.
    private void RunLoop(SimulationOptions options, IController[] controllers, TaskCompletionSource ready)
    {
        try
        {
            _driver = _driverFactory(options);
            _runner = new TwinRunner(_driver);
            foreach (IController c in controllers) _runner.Add(c);
            _runner.Start(); // warmup + OnStart
            _state = RunState.Paused;
            ready.SetResult();
        }
        catch (Exception ex)
        {
            _state = RunState.Stopped;
            ready.SetException(ex);
            return;
        }

        int maxSubsteps = Math.Max(1, options.MaxPhysicsSubstepsPerFrame);
        bool realTime = options.RealTimePlayback;
        _timeStep = Math.Max(1e-4f, options.TimeStep);  // seed live-adjustable pacing from options
        _timeScale = Math.Clamp(options.TimeScale, 0.01f, 100f);
        double accumulator = 0;
        long lastTick = Stopwatch.GetTimestamp();

        while (!_stopRequested)
        {
            try
            {
                // Drain all queued commands first (reads/writes/mode changes).
                while (_queue.TryTake(out Action? cmd)) cmd();
                if (_stopRequested) break;

                if (_state != RunState.Playing)
                {
                    Thread.Sleep(4);          // paused: yield, stay responsive to commands
                    lastTick = Stopwatch.GetTimestamp();
                    accumulator = 0;
                    continue;
                }

                int substeps = 1;
                if (realTime)
                {
                    // Advance physics to catch up to (scaled) wall-clock: accumulate elapsed time × the
                    // time-scale and run as many fixed substeps as fit, so the sim plays at the requested
                    // speed instead of slow-mo. TimeScale > 1 fills the accumulator faster → more substeps
                    // per second → accelerated sim time (bounded by physics throughput).
                    double fixedDt = Math.Max(1e-4, _timeStep);
                    double scale = _timeScale;
                    long now = Stopwatch.GetTimestamp();
                    accumulator += (now - lastTick) / (double)Stopwatch.Frequency * scale;
                    lastTick = now;
                    substeps = (int)(accumulator / fixedDt);
                    if (substeps < 1) { Thread.Sleep(1); continue; } // not enough time elapsed yet
                    // Raise the per-iteration cap with the scale so a 10× request isn't throttled by the
                    // anti-spiral clamp, while still bounding catch-up after a hitch.
                    int cap = Math.Max(maxSubsteps, (int)Math.Ceiling(scale) + 1);
                    substeps = Math.Min(substeps, cap);
                    accumulator -= substeps * fixedDt;
                    if (accumulator > cap * fixedDt) accumulator = 0; // clamp after a long hitch
                }

                StepInternal(substeps);
            }
            catch (Exception ex)
            {
                // A worker crash (or any error) during continuous play must not kill the host process.
                _fault = ex;
                break;
            }
        }

        // Drain commands queued during shutdown so their callers (Invoke/Step) don't hang forever;
        // each runs through its own try/catch wrapper, so a failure unblocks the caller with an error.
        while (_queue.TryTake(out Action? pending)) { try { pending(); } catch { /* wrapper already reported */ } }

        try { _runner?.Stop(); } catch { /* ignore */ }
        (_driver as IDisposable)?.Dispose();
        _driver = null;
        _runner = null;
        _state = _fault is null ? RunState.Stopped : RunState.Faulted;
        if (_fault is not null)
        {
            try { Faulted?.Invoke(_fault); } catch { /* never let a handler crash the sim thread */ }
        }
    }

    private volatile IController? _liveController;

    /// <summary>
    /// Installs (or clears with null) a controller that runs each frame in addition to the ones set at
    /// Start — used by the UI to run/stop a user C# script at runtime. Thread-safe. The controller's
    /// per-frame work runs on the sim thread; exceptions are swallowed so a bad script can't fault the twin.
    /// </summary>
    public void SetLiveController(IController? controller) => _liveController = controller;

    // Runs the live script controller, steps the runner one frame, refreshes the lock-free pose cache, and
    // raises FrameProduced. Always called on the sim thread.
    private void StepInternal(int physicsSubsteps = 1)
    {
        IController? live = _liveController;
        if (live is not null)
        {
            try { live.OnPreStep(_driver!.Api); } catch { /* script errors must never fault the twin */ }
        }
        IReadOnlyList<CapturedFrame> frames = _runner!.Step(physicsSubsteps);
        _latest = frames;

        // Cache rigid-body poses on the sim thread so the UI can read them lock-free (no blocking Invoke
        // per frame — that starves the UI message pump and freezes input).
        try
        {
            ISimApi api = _driver!.Api;
            IReadOnlyList<string> paths = api.RigidBodyPaths;
            float[] all = api.Read(SimTensor.RigidBodyPose, "/World/**");
            int n = Math.Min(paths.Count, all.Length / 7);
            var snapshot = new Dictionary<string, float[]>(n);
            for (int i = 0; i < n; i++)
            {
                var pose = new float[7];
                Array.Copy(all, i * 7, pose, 0, 7);
                snapshot[paths[i]] = pose;
            }
            _poseCache = snapshot;
        }
        catch { /* keep the previous snapshot if a read hiccups */ }

        FrameProduced?.Invoke(frames);
    }

    public void Play() => Post(() => { if (_state != RunState.Stopped) _state = RunState.Playing; });
    public void Pause() => Post(() => { if (_state != RunState.Stopped) _state = RunState.Paused; });

    /// <summary>Steps <paramref name="n"/> frames synchronously (regardless of play/pause), then returns.</summary>
    public void Step(int n = 1)
    {
        EnsureRunning();
        RunOnSimThread(() => { for (int i = 0; i < n && !_stopRequested; i++) StepInternal(); });
    }

    /// <summary>Runs <paramref name="action"/> on the sim thread with exclusive twin access; returns its result.</summary>
    public T Invoke<T>(Func<ISimApi, T> action)
    {
        EnsureRunning();
        T result = default!;
        RunOnSimThread(() => result = action(_driver!.Api));
        return result;
    }

    /// <summary>Runs <paramref name="action"/> on the sim thread with exclusive twin access (no result).</summary>
    public void Invoke(Action<ISimApi> action)
    {
        EnsureRunning();
        RunOnSimThread(() => action(_driver!.Api));
    }

    /// <summary>Encodes the most recent color frame for a render product as PNG (null if none yet).</summary>
    public byte[]? LatestColorPng(string? renderProduct = null)
    {
        IReadOnlyList<CapturedFrame> frames = _latest;
        CapturedFrame? frame = renderProduct is null
            ? frames.FirstOrDefault(f => f.Color is not null)
            : frames.FirstOrDefault(f => f.RenderProduct == renderProduct);
        RenderVarData? color = frame?.Color;
        if (color is null || color.Width == 0 || color.Height == 0 || color.Channels is not (3 or 4))
            return null;
        return Png.Encode(color.Bytes, color.Width, color.Height, color.Channels);
    }

    /// <summary>Signals the run loop to exit and blocks until the sim thread has torn the twin down.</summary>
    public void Stop()
    {
        if (!IsRunning) return;
        _stopRequested = true;
        _queue.Add(() => { }); // wake the loop
        _thread?.Join(10_000);
    }

    // Posts fire-and-forget work to the sim thread.
    private void Post(Action work) { if (IsRunning) _queue.Add(work); }

    // Posts work and blocks until it completes (propagating exceptions).
    private void RunOnSimThread(Action work)
    {
        if (Thread.CurrentThread == _thread) { work(); return; } // reentrant call already on the sim thread
        var done = new TaskCompletionSource();
        _queue.Add(() =>
        {
            try { work(); done.SetResult(); }
            catch (Exception ex) { done.SetException(ex); }
        });
        done.Task.GetAwaiter().GetResult();
    }

    private void EnsureRunning()
    {
        if (!IsRunning) throw new InvalidOperationException("Twin is not running; call Start first.");
    }

    public void Dispose()
    {
        Stop();
        _queue.Dispose();
    }
}
