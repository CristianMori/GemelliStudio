using System.ComponentModel;
using ModelContextProtocol.Protocol;
using ModelContextProtocol.Server;
using Gemelli.Core;
using Gemelli.Core.Control;
using Gemelli.Core.Ipc;

namespace Gemelli.Mcp;

/// <summary>
/// Agent-callable tools that drive the ovGemelli digital twin. Each method receives the shared
/// <see cref="TwinService"/> via DI; the service serializes all twin access onto its sim thread, so
/// concurrent tool calls are safe. Native library locations come from the server's environment
/// (<c>OVPHYSX_LIB</c>, <c>GEMELLI_OVRTX_DIR</c>) so the agent never needs local paths.
/// </summary>
[McpServerToolType]
public static class TwinTools
{
    [McpServerTool, Description(
        "Start the digital twin: launch the physics + render workers, load a USD scene into both, " +
        "and warm up the renderer. Returns scene info. Call this once before stepping or rendering.")]
    public static string start_twin(
        TwinService twin,
        [Description("Path (or URL) to the USD scene to load into both physics and renderer.")] string usd,
        [Description("Render product prim paths to render (e.g. a camera's RenderProduct).")] string[] renderProducts,
        [Description("Physics device: 'cpu', 'gpu', or 'auto'. Default 'auto'.")] string device = "auto",
        [Description("Rigid-body USD glob whose poses are mirrored into the renderer. Default '/World/*'.")] string rigidBodyPattern = "/World/*")
    {
        if (twin.IsRunning)
            return "Twin is already running. Call stop_twin first to load a different scene.";

        // Native library locations come from the server env, not the caller, so clients stay path-free.
        var options = new SimulationOptions
        {
            UsdPath = usd,
            RenderProducts = renderProducts,
            Device = device.ToLowerInvariant() switch
            {
                "cpu" => PhysicsDevice.Cpu,
                "gpu" => PhysicsDevice.Gpu,
                _ => PhysicsDevice.Auto,
            },
            RigidBodyPattern = rigidBodyPattern,
            OvPhysxLibrary = GemelliEnvironment.ResolveOvPhysxLibrary(),
            OvrtxLibraryDirectory = GemelliEnvironment.ResolveOvrtxDirectory(),
        };

        twin.Start(options);
        var (maj, min, patch) = twin.RenderVersion;
        return $"Twin started. ovrtx {maj}.{min}.{patch}. Rigid bodies bridged: {twin.RigidBodyPaths.Count}. " +
               $"Render products: {string.Join(", ", renderProducts)}. State: paused at t=0.";
    }

    [McpServerTool, Description("Stop the twin and shut down both workers.")]
    public static string stop_twin(TwinService twin)
    {
        if (!twin.IsRunning) return "Twin is not running.";
        twin.Stop();
        return "Twin stopped.";
    }

    [McpServerTool, Description(
        "Advance the simulation by N frames (physics + render). Returns the new simulation time and frame count.")]
    public static string step(
        TwinService twin,
        [Description("Number of frames to advance. Default 1.")] int n = 1)
    {
        twin.Step(n);
        return $"Stepped {n}. sim_time={twin.SimTime:F4}s, frame_count={twin.FrameCount}.";
    }

    [McpServerTool, Description("Get twin status: running state, simulation time, frame count, render products' rigid bodies.")]
    public static string get_info(TwinService twin)
    {
        if (!twin.IsRunning) return "Twin is not running. Call start_twin first.";
        return $"state={twin.State}, sim_time={twin.SimTime:F4}s, frame_count={twin.FrameCount}, " +
               $"rigid_bodies={twin.RigidBodyPaths.Count}.";
    }

    [McpServerTool, Description("List the prim paths of the rigid bodies bridged from physics into the renderer.")]
    public static string[] list_rigid_bodies(TwinService twin) => twin.RigidBodyPaths.ToArray();

    [McpServerTool, Description(
        "Read world poses of rigid bodies matching a USD glob. Each row is [px,py,pz, qx,qy,qz,qw] " +
        "(position + xyzw quaternion). Returns one line per body.")]
    public static string read_poses(
        TwinService twin,
        [Description("USD glob, e.g. '/World/Cube*'. Default '/World/*'.")] string pattern = "/World/*")
    {
        // Poses arrive flat: 7 floats per body (px,py,pz, qx,qy,qz,qw) — unpack into one line each.
        float[] flat = twin.Invoke(api => api.Read(SimTensor.RigidBodyPose, pattern));
        int n = flat.Length / 7;
        if (n == 0) return "(no matching bodies)";
        var lines = new List<string>(n);
        for (int i = 0; i < n; i++)
        {
            int b = i * 7;
            lines.Add($"[{i}] pos=({flat[b]:F3}, {flat[b + 1]:F3}, {flat[b + 2]:F3}) " +
                      $"quat=({flat[b + 3]:F3}, {flat[b + 4]:F3}, {flat[b + 5]:F3}, {flat[b + 6]:F3})");
        }
        return string.Join('\n', lines);
    }

    [McpServerTool, Description("Read articulation DOF positions for prims matching a USD glob (flat row-major).")]
    public static float[] read_dof(
        TwinService twin,
        [Description("USD glob selecting the articulation(s), e.g. '/World/robot'.")] string pattern)
        => twin.Invoke(api => api.Read(SimTensor.ArticulationDofPosition, pattern));

    [McpServerTool, Description("Set articulation DOF position targets for prims matching a USD glob.")]
    public static string set_dof_targets(
        TwinService twin,
        [Description("USD glob selecting the articulation(s), e.g. '/World/robot'.")] string pattern,
        [Description("Target positions, one per DOF (flat row-major).")] float[] values)
    {
        twin.Invoke(api => api.SetDofPositionTargets(pattern, values));
        return $"Set {values.Length} DOF position targets on '{pattern}'.";
    }

    [McpServerTool, Description(
        "Render and return the latest camera image as a PNG so you can SEE the current state of the " +
        "twin. Steps once if no frame has been produced yet.")]
    public static ImageContentBlock render_frame(
        TwinService twin,
        [Description("Render product path. Omit to use the first product that produced color.")] string? renderProduct = null)
    {
        if (!twin.IsRunning)
            throw new InvalidOperationException("Twin is not running. Call start_twin first.");

        byte[]? png = twin.LatestColorPng(renderProduct);
        if (png is null)
        {
            twin.Step(1); // nothing rendered yet — produce a frame
            png = twin.LatestColorPng(renderProduct);
        }
        if (png is null)
            throw new InvalidOperationException(
                "No color image available. Check the render product has a valid camera and the scene is lit.");

        return ImageContentBlock.FromBytes(png, "image/png");
    }
}
