using System.Globalization;
using System.IO.Compression;
using System.Xml.Linq;

namespace Gemelli.Fmi;

/// <summary>One scalar variable from an FMU's <c>modelDescription.xml</c>.</summary>
public sealed record FmuVariable(string Name, uint ValueReference, string Causality, double? Start);

/// <summary>
/// A live FMI 2.0 co-simulation FMU: extracts the archive to a temp directory, parses
/// <c>modelDescription.xml</c>, loads the win64 binary, and drives the co-simulation lifecycle
/// (instantiate → setup → initialize → SetReal/DoStep/GetReal → terminate). Real-valued variables
/// only — that is all the USD-FMI mappings carry.
/// </summary>
public sealed class Fmu2Instance : IDisposable
{
    private readonly string _extractDir;
    private readonly Fmi2Library _lib;
    private readonly IntPtr _component;
    private readonly Dictionary<string, FmuVariable> _variables;
    private bool _initialized;
    private bool _disposed;

    public string ModelName { get; }
    public IReadOnlyDictionary<string, FmuVariable> Variables => _variables;

    /// <summary>Loads and instantiates the FMU archive (does not enter initialization mode yet).</summary>
    public Fmu2Instance(string fmuPath, string instanceName)
    {
        if (!File.Exists(fmuPath)) throw new FmiException($"FMU not found: {fmuPath}");
        _extractDir = Path.Combine(Path.GetTempPath(), "gemelli-fmu-" + Guid.NewGuid().ToString("N"));
        ZipFile.ExtractToDirectory(fmuPath, _extractDir);

        try
        {
            XElement desc = XDocument.Load(Path.Combine(_extractDir, "modelDescription.xml")).Root
                ?? throw new FmiException($"Empty modelDescription.xml in {fmuPath}");
            string fmiVersion = (string?)desc.Attribute("fmiVersion") ?? "?";
            if (!fmiVersion.StartsWith("2.", StringComparison.Ordinal))
                throw new FmiException($"{Path.GetFileName(fmuPath)} is FMI {fmiVersion}; this host supports FMI 2.0 co-simulation.");
            ModelName = (string?)desc.Attribute("modelName") ?? Path.GetFileNameWithoutExtension(fmuPath);
            string guid = (string?)desc.Attribute("guid") ?? "";
            string modelIdentifier = (string?)desc.Element("CoSimulation")?.Attribute("modelIdentifier")
                ?? throw new FmiException($"{ModelName} has no <CoSimulation> element (model-exchange-only FMUs are not supported).");

            _variables = ParseVariables(desc);

            string dll = Path.Combine(_extractDir, "binaries", "win64", modelIdentifier + ".dll");
            if (!File.Exists(dll))
                throw new FmiException($"{ModelName} carries no win64 binary ({dll}).");
            _lib = new Fmi2Library(dll);

            string resourceUri = new Uri(Path.Combine(_extractDir, "resources") + Path.DirectorySeparatorChar).AbsoluteUri;
            _component = _lib.Instantiate(instanceName, guid, resourceUri);
        }
        catch
        {
            _lib?.Dispose();
            TryDeleteExtractDir();
            throw;
        }
    }

    private static Dictionary<string, FmuVariable> ParseVariables(XElement desc)
    {
        var vars = new Dictionary<string, FmuVariable>(StringComparer.Ordinal);
        foreach (XElement sv in desc.Element("ModelVariables")?.Elements("ScalarVariable") ?? [])
        {
            string? name = (string?)sv.Attribute("name");
            string? vrText = (string?)sv.Attribute("valueReference");
            if (name is null || vrText is null) continue;
            XElement? real = sv.Element("Real");
            if (real is null) continue; // mappings are Real-only; skip Integer/Boolean/String variables
            double? start = double.TryParse((string?)real.Attribute("start"), NumberStyles.Float, CultureInfo.InvariantCulture, out double s) ? s : null;
            vars[name] = new FmuVariable(
                name,
                uint.Parse(vrText, CultureInfo.InvariantCulture),
                (string?)sv.Attribute("causality") ?? "local",
                start);
        }
        return vars;
    }

    /// <summary>Runs setup + initialization, applying <paramref name="startValues"/> between enter and exit.</summary>
    public void Initialize(double startTime, IReadOnlyDictionary<string, double>? startValues = null)
    {
        if (_initialized) return;
        _lib.SetupExperiment(_component, startTime);
        _lib.EnterInitializationMode(_component);
        if (startValues is not null && startValues.Count > 0)
            SetReals(startValues);
        _lib.ExitInitializationMode(_component);
        _initialized = true;
    }

    /// <summary>Resolves a variable, throwing a message that lists the valid names on a miss.</summary>
    public FmuVariable Variable(string name) =>
        _variables.TryGetValue(name, out FmuVariable? v)
            ? v
            : throw new FmiException($"FMU '{ModelName}' has no Real variable '{name}'. Available: {string.Join(", ", _variables.Keys)}");

    public void SetReals(IReadOnlyDictionary<string, double> values)
    {
        if (values.Count == 0) return;
        var refs = new uint[values.Count];
        var vals = new double[values.Count];
        int i = 0;
        foreach (var (name, value) in values)
        {
            refs[i] = Variable(name).ValueReference;
            vals[i] = value;
            i++;
        }
        _lib.SetReal(_component, refs, vals);
    }

    public double GetReal(string name)
    {
        Span<uint> refs = [Variable(name).ValueReference];
        Span<double> vals = stackalloc double[1];
        _lib.GetReal(_component, refs, vals);
        return vals[0];
    }

    /// <summary>Advances the model by <paramref name="stepSize"/> from <paramref name="currentTime"/>.</summary>
    public void Step(double currentTime, double stepSize) => _lib.DoStep(_component, currentTime, stepSize);

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        try
        {
            if (_initialized) _lib.Terminate(_component);
            _lib.FreeInstance(_component);
        }
        catch { /* teardown is best effort */ }
        _lib.Dispose();
        TryDeleteExtractDir();
    }

    private void TryDeleteExtractDir()
    {
        try { Directory.Delete(_extractDir, recursive: true); } catch { /* temp dir; leak is harmless */ }
    }
}
