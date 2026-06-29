using Gemelli.Core.Sensors;

namespace Gemelli.Core.Control;

/// <summary>
/// The minimal stepping surface a <see cref="TwinRunner"/> needs. Implemented by
/// <see cref="TwinSession"/> (real two-process twin) and by fakes in tests / the UI, so the runner
/// and controllers can be driven without spawning the native workers.
/// </summary>
public interface ITwinDriver
{
    /// <summary>The control surface controllers act through.</summary>
    ISimApi Api { get; }

    /// <summary>Render warmup frames before the first real step.</summary>
    void Warmup();

    /// <summary>
    /// Advance physics by <paramref name="physicsSubsteps"/> fixed steps, then render once, returning
    /// the captured sensor frames. Substeps &gt; 1 keep physics at a fixed timestep while rendering less
    /// often (real-time pacing when the render is slower than the physics step).
    /// </summary>
    IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1);
}
