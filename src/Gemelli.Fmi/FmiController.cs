using Gemelli.Core.Control;
using Gemelli.Core.Ipc;

namespace Gemelli.Fmi;

/// <summary>
/// Runs the scene's FMI co-simulation models once per frame as a twin controller. Each step it
/// gathers inputs (cached USD attribute values, physics pose/velocity reads, overlap-sensor
/// queries), steps every FMU/SSP instance in USD authoring order, and applies the outputs
/// (articulation drive velocity targets, rigid-body forces) through <see cref="ISimApi"/>.
/// An FMI failure disables the controller and reports once — it never faults the twin.
/// </summary>
public sealed class FmiController : IController, IDisposable
{
    private sealed class LiveInstance
    {
        public required FmiInstanceConfig Config;
        public Fmu2Instance? Fmu;      // exactly one of Fmu / Ssp is set
        public SspInstanceModel? Ssp;
    }

    private readonly FmiSceneConfig _scene;
    private readonly List<LiveInstance> _instances = [];

    // {prim path -> {attr -> components}}: seeded from the stage at load; feedback outputs update it.
    private readonly Dictionary<string, Dictionary<string, double[]>> _attrCache;

    // Articulation drive routing, resolved on first use: articulation root pattern + its DOF names.
    private readonly Dictionary<string, (string Root, IReadOnlyList<string> DofNames)> _articulationForJoint = new();

    private double _time;
    private double _lastSimTime;
    private bool _failed;
    private string? _lastError;

    public FmiController(FmiSceneConfig scene)
    {
        _scene = scene;
        _attrCache = scene.InitialAttributeValues.ToDictionary(
            kv => kv.Key,
            kv => kv.Value.ToDictionary(a => a.Key, a => (double[])a.Value.Clone()));
    }

    /// <summary>Instantiates and initializes every model; start values come from the cached USD inputs.</summary>
    public void OnStart(ISimApi sim)
    {
        try
        {
            _time = sim.SimTime;
            _lastSimTime = sim.SimTime;
            foreach (FmiInstanceConfig config in _scene.Instances)
            {
                var live = new LiveInstance { Config = config };
                string name = config.PrimPath.TrimEnd('/').Split('/')[^1];
                if (config.IsSsp)
                {
                    live.Ssp = new SspInstanceModel(config.ArchivePath, name);
                    live.Ssp.Initialize(_time);
                }
                else
                {
                    live.Fmu = new Fmu2Instance(config.ArchivePath, name);
                    live.Fmu.Initialize(_time, GatherUsdInputs(config));
                }
                _instances.Add(live);
                Console.WriteLine($"[fmi] {(config.IsSsp ? "SSP" : "FMU")} '{name}' loaded from {Path.GetFileName(config.ArchivePath)}");
            }
        }
        catch (Exception ex)
        {
            Fail("start", ex);
        }
    }

    /// <summary>One co-simulation macro step, run just before the physics step of this frame.</summary>
    public void OnPreStep(ISimApi sim)
    {
        if (_failed || _instances.Count == 0) return;
        try
        {
            // The FMUs advance by the same sim-time delta the twin advanced last frame (the loop's
            // substep count isn't visible here); the first frame falls back to 1/60.
            double dt = sim.SimTime - _lastSimTime;
            if (dt <= 0) dt = 1.0 / 60.0;
            _lastSimTime = sim.SimTime;

            // Presence sensors: one sphere query per sensor per frame, shared by all instances.
            Dictionary<string, double>? overlaps = null;
            if (_scene.OverlapSensors.Count > 0)
            {
                overlaps = new Dictionary<string, double>();
                foreach (var (path, s) in _scene.OverlapSensors)
                    overlaps[path] = sim.OverlapSphere(s.X, s.Y, s.Z, s.Radius) > 0 ? 1.0 : 0.0;
            }

            var driveTargets = new Dictionary<string, double>(); // joint prim path -> rad/s

            foreach (LiveInstance live in _instances)
            {
                var inputs = GatherUsdInputs(live.Config);
                GatherPhysicsInputs(sim, live.Config, overlaps, inputs);

                IReadOnlyDictionary<string, double> outputs;
                if (live.Ssp is not null)
                {
                    outputs = live.Ssp.Step(inputs, _time, dt);
                }
                else
                {
                    live.Fmu!.SetReals(inputs);
                    live.Fmu.Step(_time, dt);
                    var read = new Dictionary<string, double>();
                    foreach (FmiConnection conn in live.Config.Connections)
                        foreach (FmiMapping m in conn.Mappings)
                            if (!m.IsInput && !read.ContainsKey(m.FmuVariable))
                                read[m.FmuVariable] = live.Fmu.GetReal(m.FmuVariable);
                    outputs = read;
                }

                ApplyOutputs(sim, live.Config, outputs, driveTargets);
            }

            if (driveTargets.Count > 0)
                WriteDriveTargets(sim, driveTargets);

            _time += dt;
        }
        catch (Exception ex)
        {
            Fail("step", ex);
        }
    }

    // Inputs sourced from cached USD attribute values (operator knobs and similar authored inputs).
    private Dictionary<string, double> GatherUsdInputs(FmiInstanceConfig config)
    {
        var inputs = new Dictionary<string, double>();
        foreach (FmiConnection conn in config.Connections)
            foreach (FmiMapping m in conn.Mappings)
            {
                if (!m.IsInput || FmiSchema.IsPhysicsAttribute(m.UsdAttribute)) continue;
                if (_attrCache.TryGetValue(conn.TargetPath, out var attrs)
                    && attrs.TryGetValue(m.UsdAttribute, out double[]? vals)
                    && m.Offset >= 0 && m.Offset < vals.Length)
                {
                    inputs[m.FmuVariable] = vals[m.Offset];
                }
            }
        return inputs;
    }

    // Inputs sourced from physics: body pose/velocity component reads and overlap-sensor presence.
    private static void GatherPhysicsInputs(
        ISimApi sim, FmiInstanceConfig config, Dictionary<string, double>? overlaps, Dictionary<string, double> into)
    {
        foreach (FmiConnection conn in config.Connections)
            foreach (FmiMapping m in conn.Mappings)
            {
                if (!m.IsInput) continue;
                switch (m.UsdAttribute)
                {
                    case FmiSchema.PhysxPosition:
                    {
                        float[] pose = sim.Read(SimTensor.RigidBodyPose, conn.TargetPath); // [px py pz qx qy qz qw]
                        if (m.Offset < 3 && pose.Length >= 3) into[m.FmuVariable] = pose[m.Offset];
                        break;
                    }
                    case FmiSchema.PhysxVelocity:
                    {
                        float[] vel = sim.Read(SimTensor.RigidBodyVelocity, conn.TargetPath); // [lin xyz, ang xyz]
                        if (m.Offset < vel.Length) into[m.FmuVariable] = vel[m.Offset];
                        break;
                    }
                    case FmiSchema.PhysxOverlap:
                        if (overlaps is not null && overlaps.TryGetValue(conn.TargetPath, out double presence))
                            into[m.FmuVariable] = presence;
                        break;
                }
            }
    }

    // Routes output mappings: drive velocities are batched per frame; forces are written immediately;
    // plain USD attribute outputs update the input cache (feedback), visual write-back is not routed.
    private void ApplyOutputs(
        ISimApi sim, FmiInstanceConfig config, IReadOnlyDictionary<string, double> outputs,
        Dictionary<string, double> driveTargets)
    {
        foreach (FmiConnection conn in config.Connections)
        {
            float[]? force = null;
            foreach (FmiMapping m in conn.Mappings)
            {
                if (m.IsInput || !outputs.TryGetValue(m.FmuVariable, out double value)) continue;
                switch (m.UsdAttribute)
                {
                    case FmiSchema.DriveTargetVelocity:
                        driveTargets[conn.TargetPath] = value;
                        break;
                    case FmiSchema.PhysxForce:
                        force ??= new float[3];
                        if (m.Offset < 3) force[m.Offset] = (float)value;
                        break;
                    default:
                        // Non-physics output: keep the attr cache in sync so input mappings reading
                        // the same attribute (feedback loops) see the new value next frame.
                        if (_attrCache.TryGetValue(conn.TargetPath, out var attrs)
                            && attrs.TryGetValue(m.UsdAttribute, out double[]? vals)
                            && m.Offset >= 0 && m.Offset < vals.Length)
                        {
                            vals[m.Offset] = value;
                        }
                        break;
                }
            }
            if (force is not null)
                sim.Write(SimTensor.RigidBodyForce, conn.TargetPath, force);
        }
    }

    // Drive targets address individual joint prims; the tensor is per-articulation. Resolve each
    // joint to its articulation root (walking up the prim path until DofNames answers), match the
    // DOF by joint leaf name, then read-modify-write the articulation's velocity-target vector.
    private void WriteDriveTargets(ISimApi sim, Dictionary<string, double> targets)
    {
        var perRoot = new Dictionary<string, Dictionary<int, double>>();
        foreach (var (jointPath, value) in targets)
        {
            if (!_articulationForJoint.TryGetValue(jointPath, out var art))
            {
                art = ResolveArticulation(sim, jointPath);
                _articulationForJoint[jointPath] = art;
            }
            if (art.Root.Length == 0) continue; // unresolved; reported once in ResolveArticulation

            string joint = jointPath.TrimEnd('/').Split('/')[^1];
            int dof = -1;
            for (int i = 0; i < art.DofNames.Count; i++)
            {
                string name = art.DofNames[i];
                if (name == joint || name.EndsWith("/" + joint, StringComparison.Ordinal)) { dof = i; break; }
            }
            if (dof < 0)
            {
                ReportOnce($"[fmi] no articulation DOF named '{joint}' under {art.Root} (DOFs: {string.Join(", ", art.DofNames)})");
                continue;
            }
            if (!perRoot.TryGetValue(art.Root, out var dofs)) perRoot[art.Root] = dofs = new Dictionary<int, double>();
            dofs[dof] = value;
        }

        foreach (var (root, dofs) in perRoot)
        {
            float[] current;
            try { current = sim.Read(SimTensor.ArticulationDofVelocityTarget, root); }
            catch { current = []; }
            int count = _articulationForJoint.Values.First(a => a.Root == root).DofNames.Count;
            if (current.Length < count) current = new float[count];
            foreach (var (dof, value) in dofs)
                if (dof < current.Length) current[dof] = (float)value;
            sim.SetDofVelocityTargets(root, current);
        }
    }

    // Finds the articulation containing a joint by asking DofNames at each ancestor path. The
    // articulation root prim is not always an ancestor of the joint: the conveyor parks its root on
    // a helper prim that is a SIBLING branch of the rollers, so each ancestor is also tried with a
    // single-level wildcard (".../*" matches the root prim wherever it hangs under that ancestor).
    private (string Root, IReadOnlyList<string> DofNames) ResolveArticulation(ISimApi sim, string jointPath)
    {
        string path = jointPath;
        while (true)
        {
            int slash = path.LastIndexOf('/');
            if (slash <= 0) break;
            path = path[..slash];
            foreach (string candidate in (string[])[path, path + "/*"])
            {
                IReadOnlyList<string> names;
                try { names = sim.DofNames(candidate); } catch { names = []; }
                if (names.Count > 0) return (candidate, names);
            }
        }
        ReportOnce($"[fmi] no articulation found for drive joint {jointPath}");
        return ("", []);
    }

    private void Fail(string phase, Exception ex)
    {
        _failed = true;
        ReportOnce($"[fmi] disabled after {phase} error: {ex.Message.Split('\n')[0]}");
    }

    private void ReportOnce(string message)
    {
        if (message == _lastError) return;
        _lastError = message;
        Console.Error.WriteLine(message);
    }

    public void OnStop(ISimApi sim) => DisposeModels();
    public void Dispose() => DisposeModels();

    private void DisposeModels()
    {
        foreach (LiveInstance live in _instances)
        {
            live.Fmu?.Dispose();
            live.Ssp?.Dispose();
        }
        _instances.Clear();
    }
}
