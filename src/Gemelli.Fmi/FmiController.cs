using Gemelli.Core.Control;
using Gemelli.Core.Ipc;

namespace Gemelli.Fmi;

/// <summary>
/// Runs the scene's FMI co-simulation models once per frame as a twin controller. Each step it
/// gathers inputs (cached USD attribute values, physics pose/velocity reads, overlap-sensor
/// queries), steps every FMU/SSP instance in USD authoring order, and applies the outputs
/// (articulation drive velocity targets, rigid-body forces) through <see cref="ISimApi"/>.
/// An FMI failure disables the controller and reports once — it never faults the twin.
///
/// The scene schema is flattened into <see cref="SignalMapping"/> rows that the signal-mapper UI
/// can observe (per-wire live values) and rewire at runtime via <see cref="AddMapping"/> /
/// <see cref="RemoveMapping"/> — all guarded by one lock, so rewiring is safe mid-run.
/// </summary>
public sealed class FmiController : IController, IDisposable
{
    private sealed class LiveInstance
    {
        public required FmiInstanceConfig Config;
        public Fmu2Instance? Fmu;      // exactly one of Fmu / Ssp is set
        public SspInstanceModel? Ssp;
        public string[] OutputVariables = [];
        public volatile IReadOnlyDictionary<string, double>? LastOutputs; // snapshot for the UI
    }

    private readonly FmiSceneConfig _scene;
    private readonly List<LiveInstance> _instances = [];

    private readonly object _mapLock = new();
    private readonly List<SignalMapping> _rows = [];
    private readonly List<FmiConstant> _constants = [];
    private int _nextId;
    private int _nextConstId;

    /// <summary>Endpoint attribute marking a wire whose source is an <see cref="FmiConstant"/>.</summary>
    public const string ConstantAttribute = "fmi:constant";

    // {prim path -> {attr -> components}}: seeded from the stage at load; feedback outputs update it.
    private readonly Dictionary<string, Dictionary<string, double[]>> _attrCache;

    // Articulation drive routing, resolved on first use: articulation root pattern + its DOF names.
    private readonly Dictionary<string, (string Root, IReadOnlyList<string> DofNames)> _articulationForJoint = new();

    private volatile IReadOnlyList<FmiInstancePorts> _ports = [];
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

        foreach (FmiInstanceConfig config in scene.Instances)
            foreach (FmiConnection conn in config.Connections)
                foreach (FmiMapping m in conn.Mappings)
                    _rows.Add(new SignalMapping
                    {
                        Id = _nextId++,
                        InstancePath = config.PrimPath,
                        FmuVariable = m.FmuVariable,
                        IsInput = m.IsInput,
                        Endpoint = new SignalEndpoint(conn.TargetPath, m.UsdAttribute, m.Offset, m.Count),
                    });
    }

    // ----- signal-mapper surface -----

    /// <summary>The scene configuration this controller was built from (sensors, instances).</summary>
    public FmiSceneConfig Scene => _scene;

    /// <summary>Connectable variables of each live instance (available once the twin has started).</summary>
    public IReadOnlyList<FmiInstancePorts> InstancePorts => _ports;

    /// <summary>Snapshot of the current wires. The returned rows are live objects: LastValue updates.</summary>
    public IReadOnlyList<SignalMapping> Mappings
    {
        get { lock (_mapLock) return _rows.ToArray(); }
    }

    /// <summary>Latest values of ALL of an instance's output variables (wired or not) — the signal
    /// mapper shows these beside unconnected output pins. Null before the first step.</summary>
    public IReadOnlyDictionary<string, double>? InstanceOutputs(string instancePath)
    {
        foreach (LiveInstance live in _instances)
            if (live.Config.PrimPath == instancePath) return live.LastOutputs;
        return null;
    }

    /// <summary>Removes a wire; takes effect on the next frame.</summary>
    public void RemoveMapping(int id)
    {
        lock (_mapLock) _rows.RemoveAll(r => r.Id == id);
    }

    /// <summary>Adds a wire between an instance variable and a scene endpoint; effective next frame.
    /// An input variable fed by several wires sees the last one in row order.</summary>
    public SignalMapping AddMapping(string instancePath, string fmuVariable, bool isInput, SignalEndpoint endpoint)
    {
        lock (_mapLock)
        {
            var row = new SignalMapping
            {
                Id = _nextId++, InstancePath = instancePath, FmuVariable = fmuVariable, IsInput = isInput, Endpoint = endpoint,
            };
            _rows.Add(row);
            return row;
        }
    }

    // ----- constants -----

    /// <summary>Live constants defined by the user (signal-mapper "const" nodes).</summary>
    public IReadOnlyList<FmiConstant> Constants
    {
        get { lock (_mapLock) return _constants.ToArray(); }
    }

    public FmiConstant AddConstant(double value = 0)
    {
        lock (_mapLock)
        {
            var c = new FmiConstant { Id = _nextConstId++, Name = $"const {_nextConstId}", Value = value };
            _constants.Add(c);
            return c;
        }
    }

    /// <summary>Removes the constant and every wire connected to it.</summary>
    public void RemoveConstant(int id)
    {
        string path = $"const:{id}";
        lock (_mapLock)
        {
            _constants.RemoveAll(c => c.Id == id);
            _rows.RemoveAll(r => r.Endpoint.UsdAttribute == ConstantAttribute && r.Endpoint.TargetPath == path
                              || r.InstancePath == path);
        }
    }

    /// <summary>Wire a constant into an FMI instance's input variable.</summary>
    public SignalMapping ConnectConstantToInput(FmiConstant c, string instancePath, string fmuVariable) =>
        AddMapping(instancePath, fmuVariable, isInput: true, new SignalEndpoint(c.Path, ConstantAttribute, 0, 0));

    /// <summary>Wire a constant straight to an actuator endpoint (drive target / force component),
    /// bypassing the FMI models entirely.</summary>
    public SignalMapping ConnectConstantToActuator(FmiConstant c, SignalEndpoint actuator) =>
        AddMapping(c.Path, "", isInput: false, actuator);

    private double? ConstantValue(string constPath)
    {
        lock (_mapLock)
        {
            foreach (FmiConstant c in _constants)
                if (c.Path == constPath) return c.Value;
        }
        return null;
    }

    // ----- controller lifecycle -----

    /// <summary>Instantiates and initializes every model; start values come from the cached USD inputs.</summary>
    public void OnStart(ISimApi sim)
    {
        try
        {
            _time = sim.SimTime;
            _lastSimTime = sim.SimTime;
            var ports = new List<FmiInstancePorts>();
            foreach (FmiInstanceConfig config in _scene.Instances)
            {
                var live = new LiveInstance { Config = config };
                string name = config.PrimPath.TrimEnd('/').Split('/')[^1];
                if (config.IsSsp)
                {
                    live.Ssp = new SspInstanceModel(config.ArchivePath, name);
                    live.Ssp.Initialize(_time);
                    live.OutputVariables = live.Ssp.OutputConnectors.ToArray();
                    ports.Add(new FmiInstancePorts(config.PrimPath, live.Ssp.SystemName, true,
                        live.Ssp.InputConnectors.ToArray(), live.OutputVariables));
                }
                else
                {
                    live.Fmu = new Fmu2Instance(config.ArchivePath, name);
                    live.Fmu.Initialize(_time, GatherUsdStartValues(config.PrimPath));
                    live.OutputVariables = live.Fmu.Variables.Values.Where(v => v.Causality == "output").Select(v => v.Name).ToArray();
                    ports.Add(new FmiInstancePorts(config.PrimPath, live.Fmu.ModelName, false,
                        live.Fmu.Variables.Values.Where(v => v.Causality == "input").Select(v => v.Name).ToArray(),
                        live.OutputVariables));
                }
                _instances.Add(live);
                Console.WriteLine($"[fmi] {(config.IsSsp ? "SSP" : "FMU")} '{name}' loaded from {Path.GetFileName(config.ArchivePath)}");
            }
            _ports = ports;
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

            SignalMapping[] rows;
            lock (_mapLock) rows = _rows.ToArray();

            // Presence sensors: one sphere query per sensor per frame, shared by all instances.
            Dictionary<string, double>? overlaps = null;
            if (_scene.OverlapSensors.Count > 0)
            {
                overlaps = new Dictionary<string, double>();
                foreach (var (path, s) in _scene.OverlapSensors)
                    overlaps[path] = sim.OverlapSphere(s.X, s.Y, s.Z, s.Radius) > 0 ? 1.0 : 0.0;
            }

            var driveTargets = new Dictionary<string, double>(); // joint prim path -> rad/s

            // Constant → actuator wires route first, so an FMI output wired to the same target wins.
            Dictionary<string, float[]>? constForces = null;
            foreach (SignalMapping row in rows)
            {
                if (row.IsInput || !row.InstancePath.StartsWith("const:", StringComparison.Ordinal)) continue;
                if (ConstantValue(row.InstancePath) is not { } value) continue;
                row.LastValue = value;
                RouteOutput(row.Endpoint, value, driveTargets, ref constForces);
            }
            if (constForces is not null)
                foreach (var (target, f) in constForces)
                    sim.Write(SimTensor.RigidBodyForce, target, f);

            foreach (LiveInstance live in _instances)
            {
                string instancePath = live.Config.PrimPath;
                var inputs = new Dictionary<string, double>();
                foreach (SignalMapping row in rows)
                {
                    if (!row.IsInput || row.InstancePath != instancePath) continue;
                    if (ReadEndpoint(sim, row.Endpoint, overlaps) is not { } value) continue;
                    row.LastValue = value;
                    inputs[row.FmuVariable] = value;
                }

                // Read EVERY declared output (not only the wired ones): the signal mapper shows
                // live values beside unconnected output pins too.
                IReadOnlyDictionary<string, double> outputs;
                if (live.Ssp is not null)
                {
                    outputs = new Dictionary<string, double>(live.Ssp.Step(inputs, _time, dt));
                }
                else
                {
                    live.Fmu!.SetReals(inputs);
                    live.Fmu.Step(_time, dt);
                    var read = new Dictionary<string, double>();
                    foreach (string v in live.OutputVariables)
                        read[v] = live.Fmu.GetReal(v);
                    outputs = read;
                }
                live.LastOutputs = outputs;

                ApplyOutputs(sim, instancePath, rows, outputs, driveTargets);
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

    // Start values for a plain FMU: every USD-sourced input wire's initial cached value.
    private Dictionary<string, double> GatherUsdStartValues(string instancePath)
    {
        var values = new Dictionary<string, double>();
        lock (_mapLock)
        {
            foreach (SignalMapping row in _rows)
            {
                if (!row.IsInput || row.InstancePath != instancePath) continue;
                if (FmiSchema.IsPhysicsAttribute(row.Endpoint.UsdAttribute)) continue;
                if (TryReadAttrCache(row.Endpoint, out double v)) values[row.FmuVariable] = v;
            }
        }
        return values;
    }

    // Resolves an input endpoint to its current value: constant, physics reads, overlap presence,
    // or cached USD.
    private double? ReadEndpoint(ISimApi sim, SignalEndpoint ep, Dictionary<string, double>? overlaps)
    {
        switch (ep.UsdAttribute)
        {
            case ConstantAttribute:
                return ConstantValue(ep.TargetPath);
            case FmiSchema.PhysxPosition:
            {
                float[] pose = sim.Read(SimTensor.RigidBodyPose, ep.TargetPath); // [px py pz qx qy qz qw]
                return ep.Offset < 3 && pose.Length >= 3 ? pose[ep.Offset] : null;
            }
            case FmiSchema.PhysxVelocity:
            {
                float[] vel = sim.Read(SimTensor.RigidBodyVelocity, ep.TargetPath); // [lin xyz, ang xyz]
                return ep.Offset < vel.Length ? vel[ep.Offset] : null;
            }
            case FmiSchema.PhysxOverlap:
                return overlaps is not null && overlaps.TryGetValue(ep.TargetPath, out double presence)
                    ? presence : null;
            default:
                return TryReadAttrCache(ep, out double v) ? v : null;
        }
    }

    private bool TryReadAttrCache(SignalEndpoint ep, out double value)
    {
        value = 0;
        if (_attrCache.TryGetValue(ep.TargetPath, out var attrs)
            && attrs.TryGetValue(ep.UsdAttribute, out double[]? vals)
            && ep.Offset >= 0 && ep.Offset < vals.Length)
        {
            value = vals[ep.Offset];
            return true;
        }
        return false;
    }

    // Routes output rows: drive velocities are batched per frame; forces are written immediately
    // (vector assembled from this instance's rows per target); plain USD attribute outputs update
    // the input cache (feedback) — visual write-back is not routed.
    private void ApplyOutputs(
        ISimApi sim, string instancePath, SignalMapping[] rows,
        IReadOnlyDictionary<string, double> outputs, Dictionary<string, double> driveTargets)
    {
        Dictionary<string, float[]>? forces = null;
        foreach (SignalMapping row in rows)
        {
            if (row.IsInput || row.InstancePath != instancePath) continue;
            if (!outputs.TryGetValue(row.FmuVariable, out double value)) continue;
            row.LastValue = value;
            RouteOutput(row.Endpoint, value, driveTargets, ref forces);
        }
        if (forces is not null)
            foreach (var (target, f) in forces)
                sim.Write(SimTensor.RigidBodyForce, target, f);
    }

    // Routes one output value to its endpoint kind: drive targets are batched by the caller, force
    // components accumulate per target, anything else updates the attribute cache (feedback only).
    private void RouteOutput(SignalEndpoint ep, double value,
        Dictionary<string, double> driveTargets, ref Dictionary<string, float[]>? forces)
    {
        switch (ep.UsdAttribute)
        {
            case FmiSchema.DriveTargetVelocity:
                driveTargets[ep.TargetPath] = value;
                break;
            case FmiSchema.PhysxForce:
                forces ??= new Dictionary<string, float[]>();
                if (!forces.TryGetValue(ep.TargetPath, out float[]? f)) forces[ep.TargetPath] = f = new float[3];
                if (ep.Offset < 3) f[ep.Offset] = (float)value;
                break;
            default:
                if (_attrCache.TryGetValue(ep.TargetPath, out var attrs)
                    && attrs.TryGetValue(ep.UsdAttribute, out double[]? vals)
                    && ep.Offset >= 0 && ep.Offset < vals.Length)
                {
                    vals[ep.Offset] = value;
                }
                break;
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
