namespace Gemelli.Core.Control;

/// <summary>
/// A live controller that drives an articulation link toward a continuously-updated world target via
/// differential IK — the basis for grabbing a robot in the viewport and dragging its end-effector. The UI
/// updates the target as the cursor moves; the controller IK-steps toward it every frame so the arm
/// converges smoothly and holds once the cursor stops.
/// </summary>
public sealed class IkDragController : IController
{
    private readonly string _robot;
    private readonly int _link;
    private readonly float _gain;
    private readonly object _lock = new();
    private float _tx, _ty, _tz;

    public IkDragController(string robot, int linkIndex, float x, float y, float z, float gain = 0.6f)
    {
        _robot = robot; _link = linkIndex; _gain = gain;
        _tx = x; _ty = y; _tz = z;
    }

    /// <summary>The link being driven (so the UI can keep its drag pivot on the same link).</summary>
    public int LinkIndex => _link;

    /// <summary>Update the world target (thread-safe; called from the UI as the cursor moves).</summary>
    public void SetTarget(float x, float y, float z)
    {
        lock (_lock) { _tx = x; _ty = y; _tz = z; }
    }

    /// <summary>Snapshots the latest target under the lock and runs one IK step toward it.</summary>
    public void OnPreStep(ISimApi sim)
    {
        float x, y, z;
        lock (_lock) { x = _tx; y = _ty; z = _tz; }
        DiffIk.StepTowards(sim, _robot, _link, x, y, z, _gain);
    }
}
