using Gemelli.Core;
using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for TwinService threading/lifecycle using an injected fake driver — no native libs.
/// Verifies single-threaded marshaling (all twin access on one thread), play/pause/step, and Invoke.
/// </summary>
public class TwinServiceTests
{
    /// <summary>Fake driver that records the managed thread each call runs on, to prove single-thread marshaling.</summary>
    private sealed class FakeDriver : ISimApi, ITwinDriver, IDisposable
    {
        public int SimThreadId = -1;
        public int WarmupThreadId = -1;
        public readonly HashSet<int> StepThreadIds = new();
        public bool Disposed;

        public double SimTime { get; private set; }
        public long FrameCount { get; private set; }
        public IReadOnlyList<string> RigidBodyPaths { get; } = ["/World/Cube1"];
        public IReadOnlyList<CapturedFrame> LatestFrames { get; private set; } = [];

        public ISimApi Api => this;
        public void Warmup() => WarmupThreadId = Environment.CurrentManagedThreadId;

        public IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1)
        {
            lock (StepThreadIds) StepThreadIds.Add(Environment.CurrentManagedThreadId);
            SimThreadId = Environment.CurrentManagedThreadId;
            SimTime += physicsSubsteps / 60.0;
            FrameCount++;
            LatestFrames = [new CapturedFrame("/Render/Cam", SimTime, SimTime, new Dictionary<string, RenderVarData>())];
            return LatestFrames;
        }

        public CapturedFrame? Frame(string product) => LatestFrames.FirstOrDefault(f => f.RenderProduct == product);
        public float[] Read(SimTensor c, string p) => [Environment.CurrentManagedThreadId];
        public (long[] Shape, float[] Data) ReadShaped(SimTensor c, string p) => ([1], Read(c, p));
        public void Write(SimTensor c, string p, float[] v) { }
        public void SetDofPositionTargets(string p, float[] v) { }
        public void SetDofVelocityTargets(string p, float[] v) { }
        public void Dispose() => Disposed = true;
    }

    // Minimal options; the fake driver ignores the USD path and render-product values.
    private static SimulationOptions Opts() => new()
    {
        UsdPath = "x.usda",
        RenderProducts = ["/Render/Cam"],
    };

    // Start warms up the driver on the sim thread and leaves the service paused (no frames yet).
    [Fact]
    public void Start_Warms_Up_And_Begins_Paused()
    {
        var fake = new FakeDriver();
        using var svc = new TwinService(_ => fake);
        svc.Start(Opts());

        Assert.True(svc.IsRunning);
        Assert.Equal(TwinService.RunState.Paused, svc.State);
        Assert.NotEqual(-1, fake.WarmupThreadId);
        Assert.Equal(0, svc.FrameCount); // paused, no steps yet
    }

    // Explicit Step(n) advances exactly n frames, all on the single sim thread.
    [Fact]
    public void Step_Advances_Frames_On_The_Sim_Thread()
    {
        var fake = new FakeDriver();
        using var svc = new TwinService(_ => fake);
        svc.Start(Opts());

        svc.Step(5);

        Assert.Equal(5, svc.FrameCount);
        // All steps ran on exactly one thread — the sim thread.
        lock (fake.StepThreadIds) Assert.Single(fake.StepThreadIds);
    }

    // Invoke runs the callback on the sim thread, not the caller's thread.
    [Fact]
    public void Invoke_Marshals_To_The_Same_Sim_Thread()
    {
        var fake = new FakeDriver();
        using var svc = new TwinService(_ => fake);
        svc.Start(Opts());
        svc.Step(1);

        int simThread = fake.SimThreadId;
        // Read returns the managed thread id it executed on; must match the sim thread, not the caller.
        float[] readThread = svc.Invoke(api => api.Read(SimTensor.RigidBodyPose, "/World/*"));

        Assert.NotEqual(Environment.CurrentManagedThreadId, (int)readThread[0]);
        Assert.Equal(simThread, (int)readThread[0]);
    }

    // Play steps continuously; Pause halts advancement and the frame count holds steady.
    [Fact]
    public void Play_Then_Pause_Steps_Continuously_Then_Stops_Advancing()
    {
        var fake = new FakeDriver();
        using var svc = new TwinService(_ => fake);
        svc.Start(Opts());

        svc.Play();
        SpinWait.SpinUntil(() => svc.FrameCount > 3, TimeSpan.FromSeconds(2));
        svc.Pause();
        // Let the loop observe the pause, then confirm the count stabilizes.
        Thread.Sleep(50);
        long afterPause = svc.FrameCount;
        Thread.Sleep(50);

        Assert.True(afterPause > 3, $"expected continuous stepping while playing, got {afterPause}");
        Assert.Equal(afterPause, svc.FrameCount); // no further steps while paused
    }

    // Stop tears down the driver (Dispose) and marks the service no longer running.
    [Fact]
    public void Stop_Disposes_Driver_And_Ends()
    {
        var fake = new FakeDriver();
        var svc = new TwinService(_ => fake);
        svc.Start(Opts());
        svc.Stop();

        Assert.False(svc.IsRunning);
        Assert.True(fake.Disposed);
        svc.Dispose();
    }

    /// <summary>Driver that throws once it has stepped past <c>failAfter</c>, to exercise fault handling.</summary>
    private sealed class ThrowingDriver : ISimApi, ITwinDriver, IDisposable
    {
        private readonly int _failAfter;
        private int _steps;
        public ThrowingDriver(int failAfter) => _failAfter = failAfter;
        public bool Disposed;

        public double SimTime { get; private set; }
        public long FrameCount { get; private set; }
        public IReadOnlyList<string> RigidBodyPaths { get; } = [];
        public IReadOnlyList<CapturedFrame> LatestFrames { get; } = [];
        public ISimApi Api => this;
        public void Warmup() { }
        public IReadOnlyList<CapturedFrame> Step(int physicsSubsteps = 1)
        {
            if (++_steps > _failAfter) throw new InvalidOperationException("simulated worker crash");
            FrameCount++; SimTime += 0.01 * physicsSubsteps;
            return [];
        }
        public CapturedFrame? Frame(string p) => null;
        public float[] Read(SimTensor c, string p) => [];
        public (long[] Shape, float[] Data) ReadShaped(SimTensor c, string p) => ([], []);
        public void Write(SimTensor c, string p, float[] v) { }
        public void SetDofPositionTargets(string p, float[] v) { }
        public void SetDofVelocityTargets(string p, float[] v) { }
        public void Dispose() => Disposed = true;
    }

    // A driver throwing mid-play transitions the service to Faulted, raises the event, and disposes the driver — host survives.
    [Fact]
    public void Worker_Crash_During_Play_Faults_Without_Killing_Host()
    {
        var driver = new ThrowingDriver(failAfter: 3);
        using var svc = new TwinService(_ => driver);
        Exception? faulted = null;
        svc.Faulted += ex => faulted = ex;

        svc.Start(Opts());
        svc.Play();

        Assert.True(SpinWait.SpinUntil(() => svc.State == TwinService.RunState.Faulted, TimeSpan.FromSeconds(3)),
            "service should transition to Faulted after the driver throws");
        Assert.NotNull(faulted);      // fault surfaced via event (host still alive)
        Assert.True(driver.Disposed); // driver torn down
        Assert.False(svc.IsRunning);
    }

    // After Stop, Start spins up a fresh driver and stepping resumes from a clean frame count.
    [Fact]
    public void Can_Restart_After_Stop()
    {
        var first = new FakeDriver();
        var second = new FakeDriver();
        int n = 0;
        using var svc = new TwinService(_ => (n++ == 0) ? first : second);

        svc.Start(Opts());
        svc.Step(2);
        svc.Stop();
        Assert.False(svc.IsRunning);

        svc.Start(Opts());                // restart with a fresh driver
        svc.Step(3);
        Assert.True(svc.IsRunning);
        Assert.Equal(3, svc.FrameCount);  // second driver stepped independently
        svc.Stop();
    }
}
