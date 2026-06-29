using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Scripting;

/// <summary>
/// The object a user behavior script (<c>.csx</c>) runs against. The script body executes once per
/// frame with these members in scope, so authors write plain C# like
/// <c>sim.SetDofPositionTargets("/World/robot", new[]{ ... });</c>.
/// </summary>
public sealed class ScriptGlobals
{
    /// <summary>The live control surface (read sensors/state, write actuation).</summary>
    public ISimApi sim { get; init; } = null!;

    /// <summary>Current frame index (0-based).</summary>
    public long frame { get; set; }

    /// <summary>Current simulation time in seconds.</summary>
    public double time { get; set; }

    /// <summary>Convenience: print a line to the host console.</summary>
    public void print(object? message) => Console.WriteLine($"[script f{frame} t{time:F3}] {message}");

    /// <summary>Convenience re-exports so scripts can use the channel enum without a using directive.</summary>
    public SimTensor DofPosition => SimTensor.ArticulationDofPosition;
    public SimTensor DofVelocity => SimTensor.ArticulationDofVelocity;
    public SimTensor LinkPose => SimTensor.ArticulationLinkPose;
    public SimTensor RigidBodyPose => SimTensor.RigidBodyPose;

    /// <summary>
    /// One differential-IK step driving the robot's TCP (link <paramref name="tcpLink"/>) toward the
    /// world target (<paramref name="x"/>,<paramref name="y"/>,<paramref name="z"/>). Call each frame to
    /// track a moving target. Returns the current TCP position (before the step), or null if unavailable.
    /// </summary>
    public (float X, float Y, float Z)? MoveTcp(string robot, int tcpLink, float x, float y, float z, float gain = 0.5f) =>
        DiffIk.StepTowards(sim, robot, tcpLink, x, y, z, gain);

    /// <summary>Reads a link's current world position from the articulation link-pose tensor.</summary>
    public (float X, float Y, float Z)? LinkPosition(string robot, int linkIndex)
    {
        var (shape, link) = sim.ReadShaped(SimTensor.ArticulationLinkPose, robot);
        if (shape.Length < 2 || link.Length < (linkIndex + 1) * 7) return null;
        int b = linkIndex * 7;
        return (link[b], link[b + 1], link[b + 2]);
    }

    /// <summary>
    /// Reads an Xbox/XInput controller (index 0–3). Use e.g. <c>var mov = ReadController();</c> then
    /// <c>mov.LeftX</c>, <c>mov.LeftY</c>, <c>mov.RightX</c>, <c>mov.RightY</c> (sticks, −1..1, deadzoned),
    /// <c>mov.LeftTrigger</c>/<c>mov.RightTrigger</c> (0..1), <c>mov.A</c>/<c>mov.B</c> (buttons), and
    /// <c>mov.Connected</c>. Returns a disconnected state if no controller is present.
    /// </summary>
    public GamepadState ReadController(int index = 0) => Gamepad.Read((uint)index);

    /// <summary>
    /// Reads the keyboard. Use <c>var kb = ReadKeyboard();</c> then <c>kb.X</c>/<c>kb.Y</c>/<c>kb.Z</c>
    /// (WASD/QE or arrows, −1..1) or <c>kb.Down("Space")</c>. Polls global key state (any thread).
    /// </summary>
    public KeyboardState ReadKeyboard() => new();

    /// <summary>True if the named key is currently held — e.g. <c>KeyDown("W")</c>, <c>KeyDown("Left")</c>.</summary>
    public bool KeyDown(string key) => Keyboard.IsDown(key);
}
