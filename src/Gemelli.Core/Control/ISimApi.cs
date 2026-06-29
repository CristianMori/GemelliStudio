using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Core.Control;

/// <summary>
/// The single control surface over a running twin. Controllers (playback, scripts, a server) read
/// sensor + physics state through this and write actuation, without knowing about processes or pipes.
/// </summary>
public interface ISimApi
{
    /// <summary>Simulation clock in seconds.</summary>
    double SimTime { get; }

    /// <summary>Number of steps taken so far.</summary>
    long FrameCount { get; }

    /// <summary>Rigid-body prim paths mirrored from physics into the render stage (tensor row order).</summary>
    IReadOnlyList<string> RigidBodyPaths { get; }

    /// <summary>Sensor frames produced by the most recent <c>Step</c> (empty before the first step).</summary>
    IReadOnlyList<CapturedFrame> LatestFrames { get; }

    /// <summary>Most recent frame for a given render product, or null.</summary>
    CapturedFrame? Frame(string renderProduct);

    /// <summary>Reads a physics tensor channel for prims matching <paramref name="pattern"/> (flat row-major).</summary>
    float[] Read(SimTensor channel, string pattern);

    /// <summary>Reads a physics tensor channel along with its shape.</summary>
    (long[] Shape, float[] Data) ReadShaped(SimTensor channel, string pattern);

    /// <summary>Names of the articulation's DOFs (joint names), in the order of the DOF-position tensor.</summary>
    IReadOnlyList<string> DofNames(string pattern) => [];

    /// <summary>Writes a physics tensor channel (e.g. DOF position/velocity targets) for matching prims.</summary>
    void Write(SimTensor channel, string pattern, float[] values);

    /// <summary>Convenience: set articulation DOF position targets.</summary>
    void SetDofPositionTargets(string pattern, float[] values);

    /// <summary>Convenience: set articulation DOF velocity targets.</summary>
    void SetDofVelocityTargets(string pattern, float[] values);
}
