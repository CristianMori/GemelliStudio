using Gemelli.Core.Sensors;

namespace Gemelli.Core.Control;

/// <summary>
/// Drives a <see cref="TwinSession"/> with a set of <see cref="IController"/>s. Each frame, every
/// controller's <see cref="IController.OnPreStep"/> runs (applying actuation through the shared
/// <see cref="ISimApi"/>) before the twin steps physics + render.
/// </summary>
public sealed class TwinRunner
{
    private readonly ITwinDriver _driver;
    private readonly List<IController> _controllers = new();
    private bool _started;

    public TwinRunner(ITwinDriver driver) => _driver = driver ?? throw new ArgumentNullException(nameof(driver));

    /// <summary>The control surface controllers act through (also usable directly).</summary>
    public ISimApi Api => _driver.Api;

    /// <summary>Registers a controller; if the runner has already started, fires its <see cref="IController.OnStart"/> now.</summary>
    public TwinRunner Add(IController controller)
    {
        ArgumentNullException.ThrowIfNull(controller);
        _controllers.Add(controller);
        if (_started) controller.OnStart(_driver.Api);
        return this;
    }

    /// <summary>Renders warmup frames and fires <see cref="IController.OnStart"/> once.</summary>
    public void Start()
    {
        if (_started) return;
        _started = true;
        _driver.Warmup();
        foreach (IController c in _controllers) c.OnStart(_driver.Api);
    }

    /// <summary>Runs controllers, then steps the twin one frame (with optional physics substeps).</summary>
    public IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1)
    {
        if (!_started) Start();
        foreach (IController c in _controllers) c.OnPreStep(_driver.Api);
        return _driver.Step(physicsSubsteps);
    }

    /// <summary>Steps <paramref name="frames"/> times, invoking <paramref name="onFrame"/> after each.</summary>
    public void Run(int frames, Action<int, IReadOnlyList<CapturedFrame>>? onFrame = null)
    {
        if (!_started) Start();
        for (int i = 0; i < frames; i++)
        {
            IReadOnlyList<CapturedFrame> result = Step();
            onFrame?.Invoke(i, result);
        }
        Stop();
    }

    /// <summary>Fires <see cref="IController.OnStop"/> for all controllers (idempotent per call).</summary>
    public void Stop()
    {
        foreach (IController c in _controllers) c.OnStop(_driver.Api);
    }
}
