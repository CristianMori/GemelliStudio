using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;
using Gemelli.Scripting;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for the control layer: TwinRunner lifecycle, ISimApi read/write, and real Roslyn
/// script compilation + execution — all against a fake driver, no native libraries required.
/// </summary>
public class ControlTests
{
    /// <summary>In-memory ISimApi + ITwinDriver: records writes, serves canned reads, counts steps.</summary>
    private sealed class FakeDriver : ISimApi, ITwinDriver
    {
        public double SimTime { get; private set; }
        public long FrameCount { get; private set; }
        public IReadOnlyList<string> RigidBodyPaths { get; } = ["/World/Cube1"];
        public IReadOnlyList<CapturedFrame> LatestFrames { get; private set; } = [];
        public int WarmupCount;
        public readonly List<(SimTensor Channel, string Pattern, float[] Values)> Writes = new();
        public readonly Dictionary<(SimTensor, string), float[]> ReadValues = new();

        public ISimApi Api => this;
        public void Warmup() => WarmupCount++;

        public IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1)
        {
            SimTime += physicsSubsteps / 60.0;
            FrameCount++;
            LatestFrames = [new CapturedFrame("/Render/Cam", SimTime, SimTime, new Dictionary<string, RenderVarData>())];
            return LatestFrames;
        }

        public CapturedFrame? Frame(string product) => LatestFrames.FirstOrDefault(f => f.RenderProduct == product);
        public float[] Read(SimTensor channel, string pattern) => ReadShaped(channel, pattern).Data;
        public (long[] Shape, float[] Data) ReadShaped(SimTensor channel, string pattern) =>
            ([1, ReadValues.TryGetValue((channel, pattern), out var v) ? v.Length : 0],
             ReadValues.GetValueOrDefault((channel, pattern), []));
        public void Write(SimTensor channel, string pattern, float[] values) => Writes.Add((channel, pattern, values));
        public void SetDofPositionTargets(string pattern, float[] values) => Write(SimTensor.ArticulationDofPositionTarget, pattern, values);
        public void SetDofVelocityTargets(string pattern, float[] values) => Write(SimTensor.ArticulationDofVelocityTarget, pattern, values);
    }

    /// <summary>IController that just tallies its lifecycle callbacks.</summary>
    private sealed class CountingController : IController
    {
        public int Starts, Steps, Stops;
        public void OnStart(ISimApi sim) => Starts++;
        public void OnPreStep(ISimApi sim) => Steps++;
        public void OnStop(ISimApi sim) => Stops++;
    }

    // Runner warms up once, then drives start / one pre-step per frame / stop in the right counts.
    [Fact]
    public void TwinRunner_Drives_Controller_Lifecycle()
    {
        var driver = new FakeDriver();
        var ctrl = new CountingController();
        var runner = new TwinRunner(driver).Add(ctrl);

        runner.Run(frames: 5);

        Assert.Equal(1, driver.WarmupCount);
        Assert.Equal(1, ctrl.Starts);
        Assert.Equal(5, ctrl.Steps);
        Assert.Equal(5, driver.FrameCount);
        Assert.Equal(1, ctrl.Stops);
    }

    // DOF target writes issued from a controller reach the driver once per frame, unmodified.
    [Fact]
    public void Controller_Can_Write_Dof_Targets_Through_Api()
    {
        var driver = new FakeDriver();
        var runner = new TwinRunner(driver).Add(new DelegateController(sim =>
            sim.SetDofPositionTargets("/World/robot", [0.1f, 0.2f, 0.3f])));

        runner.Run(frames: 2);

        Assert.Equal(2, driver.Writes.Count);
        Assert.Equal(SimTensor.ArticulationDofPositionTarget, driver.Writes[0].Channel);
        Assert.Equal([0.1f, 0.2f, 0.3f], driver.Writes[0].Values);
    }

    /// <summary>IController that forwards each pre-step to a supplied delegate.</summary>
    private sealed class DelegateController(Action<ISimApi> onPreStep) : IController
    {
        public void OnPreStep(ISimApi sim) => onPreStep(sim);
    }

    // A Roslyn-compiled script sees the frame/time globals and drives the api each tick.
    [Fact]
    public void Roslyn_Script_Compiles_And_Drives_Api()
    {
        var driver = new FakeDriver();
        // Real C# compiled by Roslyn: read a pose channel and set a DOF target based on it.
        var script = ScriptController.FromSource("""
            sim.SetDofPositionTargets("/World/arm", new[] { (float)frame, time > 0 ? 1f : 0f });
            """);

        var runner = new TwinRunner(driver).Add(script);
        runner.Run(frames: 3);

        Assert.Equal(3, driver.Writes.Count);
        Assert.Equal("/World/arm", driver.Writes[0].Pattern);
        // frame index is passed in via globals; 3rd write should carry frame==2.
        Assert.Equal(2f, driver.Writes[2].Values[0]);
    }

    // A script that throws every tick is caught; the runner keeps stepping to completion.
    [Fact]
    public void Roslyn_Script_Runtime_Error_Is_Not_Fatal()
    {
        var driver = new FakeDriver();
        // Throws on every tick; the runner must keep stepping.
        var script = ScriptController.FromSource("""throw new System.InvalidOperationException("boom");""");
        var runner = new TwinRunner(driver).Add(script);

        runner.Run(frames: 4);

        Assert.Equal(4, driver.FrameCount); // stepping continued despite the script throwing
    }
}
