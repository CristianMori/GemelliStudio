namespace Gemelli.Core.Control;

/// <summary>
/// A behavior driven each frame against the twin. Playback, embedded scripts, and a remote-control
/// server are all controllers — they read state and write actuation through <see cref="ISimApi"/>.
/// </summary>
public interface IController
{
    /// <summary>Called once before the twin is first stepped (bindings/paths are available).</summary>
    void OnStart(ISimApi sim) { }

    /// <summary>Called every frame before the physics+render step; apply actuation here.</summary>
    void OnPreStep(ISimApi sim);

    /// <summary>Called once when the run ends or the controller is removed.</summary>
    void OnStop(ISimApi sim) { }
}
