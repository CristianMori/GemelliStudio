using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace Gemelli.Fmi;

/// <summary>
/// An ONNX model (e.g. an RL locomotion policy) as a signal block: every model input becomes an
/// input pin and every model output an output pin, with pin widths taken from the tensor shapes
/// (dynamic/batch dimensions count as 1, so a [1, 60] observation input is a width-60 pin).
/// Inference runs on CPU once per frame; unwired input elements stay zero.
/// </summary>
public sealed class OnnxPolicyBlock : ISignalBlock
{
    private readonly string _modelPath;
    private InferenceSession? _session;
    private readonly List<(string Name, int[] Dims, int Width)> _inputs = [];
    private readonly List<string> _outputNames = [];

    public string DisplayName { get; private set; }
    public IReadOnlyList<BlockPin> InputPins { get; private set; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; private set; } = [];

    public OnnxPolicyBlock(string modelPath)
    {
        _modelPath = modelPath;
        DisplayName = $"ONNX  {Path.GetFileNameWithoutExtension(modelPath)}";
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues)
    {
        if (_session is not null) return;
        if (!File.Exists(_modelPath)) throw new FmiException($"ONNX model not found: {_modelPath}");
        _session = new InferenceSession(_modelPath);

        var inPins = new List<BlockPin>();
        foreach (var (name, meta) in _session.InputMetadata)
        {
            int[] dims = ConcreteDims(meta.Dimensions);
            int width = dims.Aggregate(1, (a, b) => a * b);
            _inputs.Add((name, dims, width));
            inPins.Add(new BlockPin(name, width));
        }
        var outPins = new List<BlockPin>();
        foreach (var (name, meta) in _session.OutputMetadata)
        {
            int width = ConcreteDims(meta.Dimensions).Aggregate(1, (a, b) => a * b);
            _outputNames.Add(name);
            outPins.Add(new BlockPin(name, width));
        }
        InputPins = inPins;
        OutputPins = outPins;
    }

    // Symbolic/batch dimensions (-1 or 0) become 1 so shapes are concrete for per-frame tensors.
    private static int[] ConcreteDims(int[] dims) =>
        dims.Length == 0 ? [1] : dims.Select(d => d > 0 ? d : 1).ToArray();

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        if (_session is null) return new Dictionary<string, double[]>();

        var feeds = new List<NamedOnnxValue>(_inputs.Count);
        foreach ((string name, int[] dims, int width) in _inputs)
        {
            var tensor = new DenseTensor<float>(dims);
            if (inputs.TryGetValue(name, out double[]? v))
            {
                Span<float> buf = tensor.Buffer.Span;
                for (int i = 0; i < v.Length && i < width; i++) buf[i] = (float)v[i];
            }
            feeds.Add(NamedOnnxValue.CreateFromTensor(name, tensor));
        }

        var outputs = new Dictionary<string, double[]>(_outputNames.Count);
        using IDisposableReadOnlyCollection<DisposableNamedOnnxValue> results = _session.Run(feeds);
        foreach (DisposableNamedOnnxValue r in results)
        {
            if (r.Value is not Tensor<float> t) continue;
            var values = new double[t.Length];
            int i = 0;
            foreach (float f in t) values[i++] = f;
            outputs[r.Name] = values;
        }
        return outputs;
    }

    public void Dispose() => _session?.Dispose();
}
