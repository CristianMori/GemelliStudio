using Gemelli.Core.Control;

namespace Gemelli.Fmi;

/// <summary>One connection point on a block. Width 1 is a scalar pin; wider pins carry a vector
/// whose elements can be named (<see cref="ElementLabels"/>) for the mapper's element picker.</summary>
public sealed record BlockPin(string Name, int Width = 1, IReadOnlyList<string>? ElementLabels = null);

/// <summary>
/// A behavior block in the signal graph: named input and output pins plus a per-frame step.
/// FMU and SSP instances are blocks; so are input devices (gamepad, keyboard) and, later, ONNX
/// policies. Blocks know nothing about the scene — the <see cref="SignalGraphController"/> routes
/// scene state and other blocks' outputs into <see cref="Step"/> and its results back out.
/// </summary>
public interface ISignalBlock : IDisposable
{
    /// <summary>Shown as the node title in the signal mapper.</summary>
    string DisplayName { get; }

    IReadOnlyList<BlockPin> InputPins { get; }
    IReadOnlyList<BlockPin> OutputPins { get; }

    /// <summary>Prepares the block for stepping. <paramref name="startValues"/> carries the initial
    /// value of every input pin wired to a static source (FMI start values); others may ignore it.</summary>
    void Start(double time, IReadOnlyDictionary<string, double> startValues);

    /// <summary>Advances the block by <paramref name="dt"/> and returns its output pin values.
    /// Input vectors are sized to the pin widths; missing entries mean "unwired, keep default".</summary>
    IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt);
}

/// <summary>An FMI 2.0 co-simulation FMU as a block: every Real input/output variable is a scalar pin.</summary>
public sealed class FmuBlock : ISignalBlock
{
    private readonly string _archivePath;
    private readonly string _instanceName;
    private Fmu2Instance? _fmu;

    public string DisplayName { get; private set; }
    public IReadOnlyList<BlockPin> InputPins { get; private set; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; private set; } = [];

    public FmuBlock(string archivePath, string instanceName)
    {
        _archivePath = archivePath;
        _instanceName = instanceName;
        DisplayName = $"FMU  {Path.GetFileNameWithoutExtension(archivePath)}";
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues)
    {
        _fmu = new Fmu2Instance(_archivePath, _instanceName);
        DisplayName = $"FMU  {_fmu.ModelName}";
        InputPins = _fmu.Variables.Values.Where(v => v.Causality == "input").Select(v => new BlockPin(v.Name)).ToArray();
        OutputPins = _fmu.Variables.Values.Where(v => v.Causality == "output").Select(v => new BlockPin(v.Name)).ToArray();
        _fmu.Initialize(time, startValues);
    }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        if (_fmu is null) return new Dictionary<string, double[]>();
        var scalars = new Dictionary<string, double>(inputs.Count);
        foreach (var (name, v) in inputs)
            if (v.Length > 0) scalars[name] = v[0];
        _fmu.SetReals(scalars);
        _fmu.Step(time, dt);

        var outputs = new Dictionary<string, double[]>(OutputPins.Count);
        foreach (BlockPin pin in OutputPins)
            outputs[pin.Name] = [_fmu.GetReal(pin.Name)];
        return outputs;
    }

    public void Dispose() => _fmu?.Dispose();
}

/// <summary>An SSP 1.0 archive as a block: the system-level connectors are its scalar pins.</summary>
public sealed class SspBlock : ISignalBlock
{
    private readonly string _archivePath;
    private readonly string _instanceName;
    private SspInstanceModel? _ssp;

    public string DisplayName { get; private set; }
    public IReadOnlyList<BlockPin> InputPins { get; private set; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; private set; } = [];

    public SspBlock(string archivePath, string instanceName)
    {
        _archivePath = archivePath;
        _instanceName = instanceName;
        DisplayName = $"SSP  {Path.GetFileNameWithoutExtension(archivePath)}";
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues)
    {
        _ssp = new SspInstanceModel(_archivePath, _instanceName);
        DisplayName = $"SSP  {_ssp.SystemName}";
        InputPins = _ssp.InputConnectors.Select(n => new BlockPin(n)).ToArray();
        OutputPins = _ssp.OutputConnectors.Select(n => new BlockPin(n)).ToArray();
        _ssp.Initialize(time);
    }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        if (_ssp is null) return new Dictionary<string, double[]>();
        var scalars = new Dictionary<string, double>(inputs.Count);
        foreach (var (name, v) in inputs)
            if (v.Length > 0) scalars[name] = v[0];
        // Copy: the SSP mutates and returns its internal output dictionary.
        return new Dictionary<string, double[]>(
            _ssp.Step(scalars, time, dt).ToDictionary(kv => kv.Key, kv => (double[])[kv.Value]));
    }

    public void Dispose() => _ssp?.Dispose();
}

/// <summary>
/// The Xbox controller as a source block: axes, triggers, and common buttons (as 0/1) become
/// output pins, read from XInput each step. No input pins.
/// </summary>
public sealed class GamepadBlock : ISignalBlock
{
    private static readonly string[] Pins =
        ["LeftX", "LeftY", "RightX", "RightY", "LeftTrigger", "RightTrigger", "A", "B", "LeftBumper", "RightBumper"];

    public string DisplayName => "Gamepad";
    public IReadOnlyList<BlockPin> InputPins { get; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; } = Pins.Select(p => new BlockPin(p)).ToArray();

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        GamepadState g = Gamepad.Read();
        return new Dictionary<string, double[]>
        {
            ["LeftX"] = [g.LeftX], ["LeftY"] = [g.LeftY],
            ["RightX"] = [g.RightX], ["RightY"] = [g.RightY],
            ["LeftTrigger"] = [g.LeftTrigger], ["RightTrigger"] = [g.RightTrigger],
            ["A"] = [g.A ? 1 : 0], ["B"] = [g.B ? 1 : 0],
            ["LeftBumper"] = [g.LeftBumper ? 1 : 0], ["RightBumper"] = [g.RightBumper ? 1 : 0],
        };
    }

    public void Dispose() { }
}

/// <summary>
/// The keyboard as a source block with on-the-fly pins: each added key name ("W", "Space", "Left")
/// becomes a 0/1 output pin, read from the shared <see cref="Keyboard"/> state each step. Keys can
/// be added while the twin runs (the mapper's keyboard node has an add-key box).
/// </summary>
public sealed class KeyboardBlock : ISignalBlock
{
    private readonly object _lock = new();
    private readonly List<string> _keys = ["W", "A", "S", "D"]; // useful defaults

    public string DisplayName => "Keyboard";
    public IReadOnlyList<BlockPin> InputPins { get; } = [];

    public IReadOnlyList<BlockPin> OutputPins
    {
        get { lock (_lock) return _keys.Select(k => new BlockPin(k)).ToArray(); }
    }

    /// <summary>Adds a key pin (idempotent). Returns true if the pin is new.</summary>
    public bool AddKey(string key)
    {
        key = key.Trim();
        if (key.Length == 0) return false;
        key = key.ToUpperInvariant();
        lock (_lock)
        {
            if (_keys.Contains(key)) return false;
            _keys.Add(key);
            return true;
        }
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        string[] keys;
        lock (_lock) keys = _keys.ToArray();
        var outputs = new Dictionary<string, double[]>(keys.Length);
        foreach (string k in keys)
            outputs[k] = [Keyboard.IsDown(k) ? 1 : 0];
        return outputs;
    }

    public void Dispose() { }
}
