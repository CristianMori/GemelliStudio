using Gemelli.Core.Control;
using Gemelli.Core.Ipc;

namespace Gemelli.Fmi;

/// <summary>
/// Runs the scene's signal graph once per frame as a twin controller. The graph is blocks
/// (<see cref="ISignalBlock"/>: FMU/SSP instances from the USD schema, plus runtime-added device
/// blocks) connected by <see cref="SignalMapping"/> wires to scene endpoints (sensors, USD
/// attributes, actuators) and to each other. Each step: scene-to-scene wires route first
/// (constants pinned to actuators), then every block steps in order — sources (no inputs) first —
/// with inputs gathered from the scene and from already-stepped blocks, and its outputs routed on.
/// A block failure disables the controller and reports once — it never faults the twin.
///
/// The mapper UI observes rows (per-wire live values) and rewires at runtime via
/// <see cref="Connect"/> / <see cref="RemoveMapping"/> — all guarded by one lock.
/// </summary>
public sealed class SignalGraphController : IController, IDisposable
{
    /// <summary>Pseudo endpoint attribute marking a wire whose source is an <see cref="FmiConstant"/>.</summary>
    public const string ConstantAttribute = "fmi:constant";

    private readonly FmiSceneConfig? _scene;
    private readonly List<(string Path, ISignalBlock Block)> _blocks = [];

    private readonly object _mapLock = new();
    private readonly List<SignalMapping> _rows = [];
    private readonly List<FmiConstant> _constants = [];
    private int _nextId;
    private int _nextConstId;
    private int _nextBlockId;

    // {prim path -> {attr -> components}}: seeded from the stage at load; feedback outputs update it.
    private readonly Dictionary<string, Dictionary<string, double[]>> _attrCache;

    // Articulation drive routing, resolved on first use: articulation root pattern + its DOF names.
    private readonly Dictionary<string, (string Root, IReadOnlyList<string> DofNames)> _articulationForJoint = new();

    // Latest output pin values per block path (published for the mapper's unconnected-pin labels).
    private readonly Dictionary<string, IReadOnlyDictionary<string, double[]>> _lastOutputs = new();

    private double _time;
    private double _lastSimTime;
    private bool _started;
    private bool _failed;
    private string? _lastError;

    /// <summary>Builds the graph from a scene's FMI configuration (may be null: an empty graph that
    /// only carries runtime-added blocks and constants).</summary>
    public SignalGraphController(FmiSceneConfig? scene)
    {
        _scene = scene;
        _attrCache = (scene?.InitialAttributeValues ?? new Dictionary<string, Dictionary<string, double[]>>())
            .ToDictionary(kv => kv.Key, kv => kv.Value.ToDictionary(a => a.Key, a => (double[])a.Value.Clone()));

        if (scene is null) return;
        foreach (FmiInstanceConfig config in scene.Instances)
        {
            string name = config.PrimPath.TrimEnd('/').Split('/')[^1];
            ISignalBlock block = config.Kind switch
            {
                FmiInstanceKind.Ssp => new SspBlock(config.ArchivePath, name),
                FmiInstanceKind.Onnx when config.Preset == "open_duck_mini_v2" => new DuckPolicyBlock(config.ArchivePath),
                FmiInstanceKind.Onnx => new OnnxPolicyBlock(config.ArchivePath),
                _ => new FmuBlock(config.ArchivePath, name),
            };
            _blocks.Add((config.PrimPath, block));

            foreach (FmiConnection conn in config.Connections)
                foreach (FmiMapping m in conn.Mappings)
                {
                    var ep = new SignalEndpoint(conn.TargetPath, m.UsdAttribute, m.Offset, m.Count);
                    _rows.Add(new SignalMapping
                    {
                        Id = _nextId++,
                        SourceEndpoint = m.IsInput ? ep : null,
                        SinkPin = m.IsInput ? new PinRef(config.PrimPath, m.FmuVariable) : null,
                        SourcePin = m.IsInput ? null : new PinRef(config.PrimPath, m.FmuVariable),
                        SinkEndpoint = m.IsInput ? null : ep,
                        // The schema's (offset, count) selector doubles as the wire width: a count
                        // above 1 is a vector wire (scene vector endpoints or block vector pins).
                        Count = Math.Max(1, m.Count),
                    });
                }
        }
    }

    // ----- mapper surface: blocks -----

    /// <summary>Every block in the graph with its path key ("/World/..." for scene instances,
    /// "block:N" for runtime-added devices), in execution order.</summary>
    public IReadOnlyList<(string Path, ISignalBlock Block)> Blocks
    {
        get { lock (_mapLock) return _blocks.ToArray(); }
    }

    /// <summary>Adds a runtime block (device, policy). Safe while the twin runs.</summary>
    public string AddBlock(ISignalBlock block)
    {
        lock (_mapLock)
        {
            string path = $"block:{_nextBlockId++}";
            _blocks.Add((path, block));
            if (_started) block.Start(_time, new Dictionary<string, double>());
            return path;
        }
    }

    /// <summary>Removes a runtime block and every wire touching it (scene instances stay).</summary>
    public void RemoveBlock(string path)
    {
        if (!path.StartsWith("block:", StringComparison.Ordinal)) return;
        ISignalBlock? removed = null;
        lock (_mapLock)
        {
            int i = _blocks.FindIndex(b => b.Path == path);
            if (i < 0) return;
            removed = _blocks[i].Block;
            _blocks.RemoveAt(i);
            _rows.RemoveAll(r => r.SourcePin?.BlockPath == path || r.SinkPin?.BlockPath == path);
            _lastOutputs.Remove(path);
        }
        removed?.Dispose();
    }

    /// <summary>Latest values of ALL of a block's output pins (wired or not). Null before it steps.</summary>
    public IReadOnlyDictionary<string, double[]>? BlockOutputs(string path)
    {
        lock (_mapLock) return _lastOutputs.GetValueOrDefault(path);
    }

    // ----- mapper surface: wires -----

    /// <summary>Snapshot of the current wires. The returned rows are live objects: LastValues update.</summary>
    public IReadOnlyList<SignalMapping> Mappings
    {
        get { lock (_mapLock) return _rows.ToArray(); }
    }

    public void RemoveMapping(int id)
    {
        lock (_mapLock) _rows.RemoveAll(r => r.Id == id);
    }

    /// <summary>
    /// Adds a wire; effective next frame. Exactly one source (scene endpoint or block output pin)
    /// and one sink (block input pin or scene endpoint) must be given. Offsets select elements when
    /// the sides have different widths; <paramref name="count"/> is the number of elements carried.
    /// An input pin fed by several wires sees them applied in row order (last write wins per element).
    /// </summary>
    public SignalMapping Connect(
        SignalEndpoint? sourceEndpoint, PinRef? sourcePin,
        PinRef? sinkPin, SignalEndpoint? sinkEndpoint,
        int sourceOffset = 0, int sinkOffset = 0, int count = 1)
    {
        if (sourceEndpoint is null == sourcePin is null || sinkPin is null == sinkEndpoint is null)
            throw new ArgumentException("A wire needs exactly one source and one sink.");
        lock (_mapLock)
        {
            var row = new SignalMapping
            {
                Id = _nextId++,
                SourceEndpoint = sourceEndpoint, SourcePin = sourcePin,
                SinkPin = sinkPin, SinkEndpoint = sinkEndpoint,
                SourceOffset = sourceOffset, SinkOffset = sinkOffset, Count = Math.Max(1, count),
            };
            _rows.Add(row);
            return row;
        }
    }

    // ----- mapper surface: constants -----

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
            _rows.RemoveAll(r => r.SourceEndpoint is { } ep && ep.UsdAttribute == ConstantAttribute && ep.TargetPath == path);
        }
    }

    /// <summary>The endpoint a constant's wires use as their source.</summary>
    public static SignalEndpoint ConstantEndpoint(FmiConstant c) => new(c.Path, ConstantAttribute, 0, 0);

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

    /// <summary>Starts every block; FMU start values come from the wires into it (static sources only).</summary>
    public void OnStart(ISimApi sim)
    {
        try
        {
            _time = sim.SimTime;
            _lastSimTime = sim.SimTime;
            (string Path, ISignalBlock Block)[] blocks;
            lock (_mapLock) blocks = _blocks.ToArray();
            foreach ((string path, ISignalBlock block) in blocks)
            {
                block.Start(_time, GatherStartValues(path));
                Console.WriteLine($"[fmi] block '{block.DisplayName}' ready ({path})");
            }
            _started = true;
        }
        catch (Exception ex)
        {
            Fail("start", ex);
        }
    }

    /// <summary>One signal-graph step, run just before the physics step of this frame.</summary>
    public void OnPreStep(ISimApi sim)
    {
        if (_failed) return;
        try
        {
            // The blocks advance by the same sim-time delta the twin advanced last frame (the loop's
            // substep count isn't visible here); the first frame falls back to 1/60.
            double dt = sim.SimTime - _lastSimTime;
            if (dt <= 0) dt = 1.0 / 60.0;
            _lastSimTime = sim.SimTime;

            SignalMapping[] rows;
            (string Path, ISignalBlock Block)[] blocks;
            lock (_mapLock)
            {
                rows = _rows.ToArray();
                blocks = _blocks.ToArray();
            }

            // Presence sensors: one sphere query per sensor per frame, shared by every wire.
            Dictionary<string, double>? overlaps = null;
            if (_scene is not null && _scene.OverlapSensors.Count > 0)
            {
                overlaps = new Dictionary<string, double>();
                foreach (var (path, s) in _scene.OverlapSensors)
                    overlaps[path] = sim.OverlapSphere(s.X, s.Y, s.Z, s.Radius) > 0 ? 1.0 : 0.0;
            }

            var sinks = new ActuatorSinks();

            // Scene-to-scene wires first (constants or sensors pinned straight to actuators), so a
            // block output wired to the same target wins.
            foreach (SignalMapping row in rows)
            {
                if (row.SourceEndpoint is null || row.SinkEndpoint is null) continue;
                if (ReadSource(sim, row, overlaps) is not { } value) continue;
                row.LastValues = value;
                RouteToEndpoint(sim, row.SinkEndpoint, row.SinkOffset, value, sinks);
            }

            // Blocks in execution order: sources (no input pins) first, then insertion order — so
            // device readings reach consumers within the same frame.
            foreach ((string path, ISignalBlock block) in blocks.OrderBy(b => b.Block.InputPins.Count > 0 ? 1 : 0))
            {
                var inputs = new Dictionary<string, double[]>();
                foreach (SignalMapping row in rows)
                {
                    if (row.SinkPin is null || row.SinkPin.BlockPath != path) continue;
                    if (ReadSource(sim, row, overlaps) is not { } value) continue;
                    row.LastValues = value;
                    WriteIntoPinBuffer(block, row.SinkPin.Pin, row.SinkOffset, value, inputs);
                }

                IReadOnlyDictionary<string, double[]> outputs = block.Step(inputs, _time, dt);
                lock (_mapLock) _lastOutputs[path] = outputs;

                foreach (SignalMapping row in rows)
                {
                    if (row.SourcePin is null || row.SourcePin.BlockPath != path || row.SinkEndpoint is null) continue;
                    if (SliceFromOutputs(outputs, row) is not { } value) continue;
                    row.LastValues = value;
                    RouteToEndpoint(sim, row.SinkEndpoint, row.SinkOffset, value, sinks);
                }
            }

            sinks.Flush(sim, this);
            _time += dt;
        }
        catch (Exception ex)
        {
            Fail("step", ex);
        }
    }

    // Start values for a block: every wired input pin whose source is static (USD attr or constant).
    private Dictionary<string, double> GatherStartValues(string blockPath)
    {
        var values = new Dictionary<string, double>();
        lock (_mapLock)
        {
            foreach (SignalMapping row in _rows)
            {
                if (row.SinkPin is null || row.SinkPin.BlockPath != blockPath || row.SourceEndpoint is null) continue;
                SignalEndpoint ep = row.SourceEndpoint;
                if (ep.UsdAttribute == ConstantAttribute)
                {
                    if (ConstantValue(ep.TargetPath) is { } cv) values[row.SinkPin.Pin] = cv;
                }
                else if (!FmiSchema.IsPhysicsAttribute(ep.UsdAttribute) && TryReadAttrCache(ep, out double v))
                {
                    values[row.SinkPin.Pin] = v;
                }
            }
        }
        return values;
    }

    // ----- signal resolution -----

    // The value(s) a wire carries this frame, from whichever source kind it has.
    private double[]? ReadSource(ISimApi sim, SignalMapping row, Dictionary<string, double>? overlaps)
    {
        if (row.SourcePin is { } pin)
        {
            IReadOnlyDictionary<string, double[]>? outputs;
            lock (_mapLock) outputs = _lastOutputs.GetValueOrDefault(pin.BlockPath);
            return outputs is null ? null : SliceVector(outputs.GetValueOrDefault(pin.Pin), row.SourceOffset, row.Count);
        }

        SignalEndpoint ep = row.SourceEndpoint!;
        switch (ep.UsdAttribute)
        {
            case ConstantAttribute:
                return ConstantValue(ep.TargetPath) is { } cv ? [cv] : null;
            case FmiSchema.PhysxPosition:
            {
                float[] pose = sim.Read(SimTensor.RigidBodyPose, ep.TargetPath); // [px py pz qx qy qz qw]
                return ep.Offset < 3 && pose.Length >= 3 ? [pose[ep.Offset]] : null;
            }
            case FmiSchema.PhysxVelocity:
            {
                float[] vel = sim.Read(SimTensor.RigidBodyVelocity, ep.TargetPath); // [lin xyz, ang xyz]
                return ep.Offset < vel.Length ? [vel[ep.Offset]] : null;
            }
            case FmiSchema.PhysxOverlap:
                return overlaps is not null && overlaps.TryGetValue(ep.TargetPath, out double presence)
                    ? [presence] : null;
            case FmiSchema.DofPositions:
            case FmiSchema.DofVelocities:
            {
                // Whole articulation DOF vector (measured), sliced by the wire's element selection.
                SimTensor channel = ep.UsdAttribute == FmiSchema.DofPositions
                    ? SimTensor.ArticulationDofPosition : SimTensor.ArticulationDofVelocity;
                return SliceFloats(sim.Read(channel, ep.TargetPath), row);
            }
            case FmiSchema.BodyPose:
                return SliceFloats(sim.Read(SimTensor.RigidBodyPose, ep.TargetPath), row);
            case FmiSchema.BodyVelocity:
                return SliceFloats(sim.Read(SimTensor.RigidBodyVelocity, ep.TargetPath), row);
            default:
                return TryReadAttrCache(ep, out double v) ? [v] : null;
        }
    }

    // Slices a block output pin's vector for a wire (SourceOffset, Count).
    private double[]? SliceFromOutputs(IReadOnlyDictionary<string, double[]> outputs, SignalMapping row) =>
        SliceVector(outputs.GetValueOrDefault(row.SourcePin!.Pin), row.SourceOffset, row.Count);

    private static double[]? SliceFloats(float[] values, SignalMapping row)
    {
        var all = new double[values.Length];
        for (int i = 0; i < values.Length; i++) all[i] = values[i];
        return SliceVector(all, row.SourceOffset, row.Count);
    }

    private static double[]? SliceVector(double[]? vector, int offset, int count)
    {
        if (vector is null || offset < 0 || offset >= vector.Length) return null;
        int n = Math.Min(count, vector.Length - offset);
        if (n == vector.Length && offset == 0) return vector;
        double[] slice = new double[n];
        Array.Copy(vector, offset, slice, 0, n);
        return slice;
    }

    // Places a wire's value into a block's input buffer at the sink offset, sized to the pin width.
    private static void WriteIntoPinBuffer(
        ISignalBlock block, string pinName, int sinkOffset, double[] value, Dictionary<string, double[]> buffers)
    {
        int width = 1;
        foreach (BlockPin p in block.InputPins)
            if (p.Name == pinName) { width = Math.Max(1, p.Width); break; }
        if (!buffers.TryGetValue(pinName, out double[]? buf) || buf.Length != width)
            buffers[pinName] = buf = new double[width];
        for (int i = 0; i < value.Length && sinkOffset + i < buf.Length; i++)
            buf[sinkOffset + i] = value[i];
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

    // ----- actuator routing -----

    // Actuator writes batched within one frame: scalar per-joint drive targets, whole-articulation
    // DOF target vectors, and per-body force vectors, all flushed once after the blocks step.
    private sealed class ActuatorSinks
    {
        public readonly Dictionary<string, double> JointVelocity = new();                 // joint prim -> rad/s
        public readonly Dictionary<string, (bool Position, Dictionary<int, double> Dofs)> DofVectors = new();
        public readonly Dictionary<string, float[]> Forces = new();                       // body prim -> [fx fy fz]

        public void Flush(ISimApi sim, SignalGraphController c)
        {
            foreach (var (target, f) in Forces)
                sim.Write(SimTensor.RigidBodyForce, target, f);
            if (JointVelocity.Count > 0)
                c.WriteJointVelocityTargets(sim, JointVelocity);
            foreach (var (pattern, (position, dofs)) in DofVectors)
                c.WriteDofVector(sim, pattern, position, dofs);
        }
    }

    // Routes one wire's value to its sink endpoint kind.
    private void RouteToEndpoint(ISimApi sim, SignalEndpoint ep, int sinkOffset, double[] value, ActuatorSinks sinks)
    {
        switch (ep.UsdAttribute)
        {
            case FmiSchema.DriveTargetVelocity:
                if (value.Length > 0) sinks.JointVelocity[ep.TargetPath] = value[0];
                break;

            case FmiSchema.DofVelocityTargets:
            case FmiSchema.DofPositionTargets:
            {
                bool position = ep.UsdAttribute == FmiSchema.DofPositionTargets;
                if (!sinks.DofVectors.TryGetValue(ep.TargetPath, out var entry) || entry.Position != position)
                    sinks.DofVectors[ep.TargetPath] = entry = (position, new Dictionary<int, double>());
                for (int i = 0; i < value.Length; i++)
                    entry.Dofs[sinkOffset + i] = value[i];
                break;
            }

            case FmiSchema.PhysxForce:
                if (!sinks.Forces.TryGetValue(ep.TargetPath, out float[]? f)) sinks.Forces[ep.TargetPath] = f = new float[3];
                if (ep.Offset < 3 && value.Length > 0) f[ep.Offset] = (float)value[0];
                break;

            default:
                // Non-physics output: keep the attr cache in sync so input wires reading the same
                // attribute (feedback loops) see the new value next frame.
                if (value.Length > 0
                    && _attrCache.TryGetValue(ep.TargetPath, out var attrs)
                    && attrs.TryGetValue(ep.UsdAttribute, out double[]? vals)
                    && ep.Offset >= 0 && ep.Offset < vals.Length)
                {
                    vals[ep.Offset] = value[0];
                }
                break;
        }
    }

    // Per-joint drive targets address individual joint prims; the tensor is per-articulation.
    // Resolve each joint to its articulation, match the DOF by joint leaf name, then
    // read-modify-write the articulation's velocity-target vector.
    private void WriteJointVelocityTargets(ISimApi sim, Dictionary<string, double> targets)
    {
        var perRoot = new Dictionary<string, Dictionary<int, double>>();
        foreach (var (jointPath, value) in targets)
        {
            (string root, IReadOnlyList<string> dofNames) = ArticulationForJoint(sim, jointPath);
            if (root.Length == 0) continue; // unresolved; reported once

            string joint = jointPath.TrimEnd('/').Split('/')[^1];
            int dof = -1;
            for (int i = 0; i < dofNames.Count; i++)
                if (dofNames[i] == joint || dofNames[i].EndsWith("/" + joint, StringComparison.Ordinal)) { dof = i; break; }
            if (dof < 0)
            {
                ReportOnce($"[fmi] no articulation DOF named '{joint}' under {root} (DOFs: {string.Join(", ", dofNames)})");
                continue;
            }
            if (!perRoot.TryGetValue(root, out var dofs)) perRoot[root] = dofs = new Dictionary<int, double>();
            dofs[dof] = value;
        }
        foreach (var (root, dofs) in perRoot)
            WriteDofVector(sim, root, position: false, dofs);
    }

    // Read-modify-write of an articulation's DOF target vector (velocity or position).
    private void WriteDofVector(ISimApi sim, string pattern, bool position, Dictionary<int, double> dofs)
    {
        SimTensor channel = position ? SimTensor.ArticulationDofPositionTarget : SimTensor.ArticulationDofVelocityTarget;
        float[] current;
        try { current = sim.Read(channel, pattern); }
        catch { current = []; }
        int count;
        try { count = sim.DofNames(pattern).Count; } catch { count = current.Length; }
        if (current.Length < count) current = new float[count];
        foreach (var (dof, value) in dofs)
            if (dof >= 0 && dof < current.Length) current[dof] = (float)value;
        if (position) sim.SetDofPositionTargets(pattern, current);
        else sim.SetDofVelocityTargets(pattern, current);
    }

    // Finds the articulation containing a joint by asking DofNames at each ancestor path. The
    // articulation root prim is not always an ancestor of the joint: the conveyor parks its root on
    // a helper prim that is a SIBLING branch of the rollers, so each ancestor is also tried with a
    // single-level wildcard (".../*" matches the root prim wherever it hangs under that ancestor).
    private (string Root, IReadOnlyList<string> DofNames) ArticulationForJoint(ISimApi sim, string jointPath)
    {
        if (_articulationForJoint.TryGetValue(jointPath, out var cached)) return cached;

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
                if (names.Count > 0) return _articulationForJoint[jointPath] = (candidate, names);
            }
        }
        ReportOnce($"[fmi] no articulation found for drive joint {jointPath}");
        return _articulationForJoint[jointPath] = ("", []);
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

    public void OnStop(ISimApi sim) => DisposeBlocks();
    public void Dispose() => DisposeBlocks();

    private void DisposeBlocks()
    {
        (string Path, ISignalBlock Block)[] blocks;
        lock (_mapLock)
        {
            blocks = _blocks.ToArray();
            _blocks.Clear();
        }
        foreach ((_, ISignalBlock block) in blocks) block.Dispose();
    }
}
