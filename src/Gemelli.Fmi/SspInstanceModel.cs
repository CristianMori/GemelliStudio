using System.IO.Compression;
using System.Xml.Linq;

namespace Gemelli.Fmi;

/// <summary>
/// A live SSP 1.0 archive: a black box wrapping several internally-wired FMUs. Only the system-level
/// connectors are visible to callers; internal FMU-to-FMU connections stay encapsulated, matching the
/// ovfmi <c>SspInstance</c> semantics. Components step once per <see cref="Step"/> in the order they
/// are declared in <c>SystemStructure.ssd</c> (which encodes causality), with outputs propagated to
/// downstream inputs within the same macro step.
/// </summary>
public sealed class SspInstanceModel : IDisposable
{
    private sealed record Connection(string? FromElement, string FromConnector, string? ToElement, string ToConnector);

    private readonly string _extractDir;
    private readonly List<(string Name, Fmu2Instance Fmu)> _components = [];
    private readonly List<Connection> _connections = [];
    private readonly Dictionary<string, string> _systemInputKinds = new();   // connector -> kind
    private readonly Dictionary<string, double> _systemOutputs = new();      // latest system-connector outputs
    private bool _disposed;

    public string SystemName { get; }

    /// <summary>System-level connector names by kind, for validation and diagnostics.</summary>
    public IReadOnlyCollection<string> InputConnectors { get; }
    public IReadOnlyCollection<string> OutputConnectors { get; }

    public SspInstanceModel(string sspPath, string instanceName)
    {
        if (!File.Exists(sspPath)) throw new FmiException($"SSP not found: {sspPath}");
        _extractDir = Path.Combine(Path.GetTempPath(), "gemelli-ssp-" + Guid.NewGuid().ToString("N"));
        ZipFile.ExtractToDirectory(sspPath, _extractDir);

        try
        {
            XElement root = XDocument.Load(Path.Combine(_extractDir, "SystemStructure.ssd")).Root
                ?? throw new FmiException($"Empty SystemStructure.ssd in {sspPath}");
            XElement system = ElementByLocalName(root, "System")
                ?? throw new FmiException($"SystemStructure.ssd in {sspPath} has no <ssd:System>.");
            SystemName = (string?)system.Attribute("name") ?? Path.GetFileNameWithoutExtension(sspPath);

            var inputs = new List<string>();
            var outputs = new List<string>();
            foreach (XElement conn in ElementsByLocalName(ElementByLocalName(system, "Connectors"), "Connector"))
            {
                string name = (string?)conn.Attribute("name") ?? "";
                string kind = (string?)conn.Attribute("kind") ?? "";
                _systemInputKinds[name] = kind;
                (kind == "input" ? inputs : outputs).Add(name);
            }
            InputConnectors = inputs;
            OutputConnectors = outputs;

            // Instantiate internal FMUs in declaration order — that order IS the execution order.
            foreach (XElement comp in ElementsByLocalName(ElementByLocalName(system, "Elements"), "Component"))
            {
                string name = (string?)comp.Attribute("name") ?? throw new FmiException("SSP component without a name.");
                string source = (string?)comp.Attribute("source") ?? throw new FmiException($"SSP component '{name}' without a source.");
                string fmuPath = Path.Combine(_extractDir, source.Replace('/', Path.DirectorySeparatorChar));
                _components.Add((name, new Fmu2Instance(fmuPath, $"{instanceName}.{name}")));
            }

            foreach (XElement c in ElementsByLocalName(ElementByLocalName(system, "Connections"), "Connection"))
            {
                _connections.Add(new Connection(
                    (string?)c.Attribute("startElement"),
                    (string?)c.Attribute("startConnector") ?? "",
                    (string?)c.Attribute("endElement"),
                    (string?)c.Attribute("endConnector") ?? ""));
            }
        }
        catch
        {
            Dispose();
            throw;
        }
    }

    // .ssd files use ssd:/ssc: namespaces; match by local name so we do not depend on prefix spelling.
    private static XElement? ElementByLocalName(XElement? parent, string localName) =>
        parent?.Elements().FirstOrDefault(e => e.Name.LocalName == localName);
    private static IEnumerable<XElement> ElementsByLocalName(XElement? parent, string localName) =>
        parent?.Elements().Where(e => e.Name.LocalName == localName) ?? [];

    /// <summary>Initializes every internal FMU (start values come from each FMU's own defaults).</summary>
    public void Initialize(double startTime)
    {
        foreach ((_, Fmu2Instance fmu) in _components)
            fmu.Initialize(startTime);
    }

    /// <summary>
    /// One atomic macro step: applies system inputs, steps each component in declaration order
    /// (propagating its outputs to downstream inputs and system outputs immediately), and returns
    /// the latest system-connector output values.
    /// </summary>
    public IReadOnlyDictionary<string, double> Step(
        IReadOnlyDictionary<string, double> systemInputs, double currentTime, double stepSize)
    {
        // System input connector -> every component input it feeds.
        foreach (Connection c in _connections)
        {
            if (c.FromElement is not null || c.ToElement is null) continue;
            if (!systemInputs.TryGetValue(c.FromConnector, out double value)) continue;
            ComponentByName(c.ToElement).SetReals(new Dictionary<string, double> { [c.ToConnector] = value });
        }

        foreach ((string name, Fmu2Instance fmu) in _components)
        {
            fmu.Step(currentTime, stepSize);

            // Push this component's outputs downstream (later components see them this same step)
            // and surface the ones wired to system output connectors.
            foreach (Connection c in _connections)
            {
                if (c.FromElement != name) continue;
                double value = fmu.GetReal(c.FromConnector);
                if (c.ToElement is null)
                    _systemOutputs[c.ToConnector] = value;
                else
                    ComponentByName(c.ToElement).SetReals(new Dictionary<string, double> { [c.ToConnector] = value });
            }
        }
        return _systemOutputs;
    }

    private Fmu2Instance ComponentByName(string name)
    {
        foreach ((string n, Fmu2Instance fmu) in _components)
            if (n == name) return fmu;
        throw new FmiException($"SSP '{SystemName}' has no component '{name}'.");
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        foreach ((_, Fmu2Instance fmu) in _components) fmu.Dispose();
        _components.Clear();
        try { Directory.Delete(_extractDir, recursive: true); } catch { /* temp dir; leak is harmless */ }
    }
}
