namespace Gemelli.Core.Control;

/// <summary>
/// The simplest controller: it applies no actuation, letting physics (and any USD time-keyed
/// animation in the workers) play out on its own. Useful as a baseline and for pure capture runs.
/// </summary>
public sealed class PlaybackController : IController
{
    public void OnPreStep(ISimApi sim) { /* passive playback — no actuation */ }
}
