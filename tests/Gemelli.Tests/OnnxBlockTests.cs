using Gemelli.Fmi;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tests for the ONNX policy block, driven against the Open Duck Mini walking policy when its
/// checkout is present on this machine (skipped otherwise — the model is not part of this repo).
/// </summary>
public class OnnxBlockTests(Xunit.Abstractions.ITestOutputHelper output)
{
    private const string DuckPolicy = @"C:\DataDrive\Open_Duck_Mini\BEST_WALK_ONNX_2.onnx";

    [SkippableFact]
    public void Duck_Policy_Loads_And_Steps()
    {
        Skip.IfNot(File.Exists(DuckPolicy), "Open_Duck_Mini checkout with BEST_WALK_ONNX_2.onnx not found.");

        using var block = new OnnxPolicyBlock(DuckPolicy);
        block.Start(0, new Dictionary<string, double>());

        foreach (BlockPin p in block.InputPins) output.WriteLine($"in  {p.Name} [{p.Width}]");
        foreach (BlockPin p in block.OutputPins) output.WriteLine($"out {p.Name} [{p.Width}]");

        Assert.NotEmpty(block.InputPins);
        Assert.NotEmpty(block.OutputPins);

        // Feed zeros (all pins unwired) and confirm inference produces finitely-valued outputs of
        // the declared widths — the plumbing contract every policy block relies on.
        IReadOnlyDictionary<string, double[]> outputs =
            block.Step(new Dictionary<string, double[]>(), 0, 1.0 / 50);

        foreach (BlockPin pin in block.OutputPins)
        {
            double[] v = Assert.Contains(pin.Name, outputs);
            Assert.Equal(pin.Width, v.Length);
            Assert.All(v, x => Assert.True(double.IsFinite(x)));
        }
    }
}
