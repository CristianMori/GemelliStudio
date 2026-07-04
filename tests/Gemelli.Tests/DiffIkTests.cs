using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for the damped-least-squares IK step, driven through a canned <see cref="ISimApi"/>.
/// The Jacobian is chosen so the expected joint deltas have a closed form:
/// with the TCP's linear rows = [[1,0],[0,1],[0,0]], A = J·Jᵀ + λ²I is diagonal and
/// dq = (ex, ey) / (1 + λ²) exactly.
/// </summary>
public class DiffIkTests
{
    private const string Robot = "/World/robot";
    private const int TcpLink = 2;   // of 3 links; fixed base → Jacobian block = TcpLink − 1
    private const int Cols = 2;      // DOFs

    /// <summary>Serves canned link poses / Jacobian / DOF targets; records target writes.</summary>
    private sealed class FakeSim : ISimApi
    {
        public float[] LinkPoses = new float[3 * 7];         // [1, 3, 7]
        public long[] LinkShape = [1, 3, 7];
        public float[] Jacobian = new float[12 * Cols];      // [1, 12, 2]: 2 moving links × 6 rows
        public long[] JacobianShape = [1, 12, Cols];
        public float[] DofTargets = new float[Cols];
        public readonly List<(string Pattern, float[] Values)> TargetWrites = new();

        public double SimTime => 0;
        public long FrameCount => 0;
        public IReadOnlyList<string> RigidBodyPaths => [];
        public IReadOnlyList<CapturedFrame> LatestFrames => [];
        public CapturedFrame? Frame(string p) => null;

        public float[] Read(SimTensor c, string p) => ReadShaped(c, p).Data;
        public (long[] Shape, float[] Data) ReadShaped(SimTensor c, string p) => c switch
        {
            SimTensor.ArticulationLinkPose => (LinkShape, LinkPoses),
            SimTensor.ArticulationJacobian => (JacobianShape, Jacobian),
            SimTensor.ArticulationDofPositionTarget => ([1, Cols], DofTargets),
            _ => ([], []),
        };

        public void Write(SimTensor c, string p, float[] v) { }
        public void SetDofPositionTargets(string p, float[] v) => TargetWrites.Add((p, v));
        public void SetDofVelocityTargets(string p, float[] v) { }
    }

    // A fake posed with the TCP at (1, 2, 3) and identity-like linear Jacobian rows for the TCP block.
    private static FakeSim Posed()
    {
        var sim = new FakeSim();
        sim.LinkPoses[TcpLink * 7 + 0] = 1f;
        sim.LinkPoses[TcpLink * 7 + 1] = 2f;
        sim.LinkPoses[TcpLink * 7 + 2] = 3f;
        // Fixed base: block = TcpLink − 1 = 1 → linear rows 6..8. Rows 6,7 map DOF 0,1; row 8 zero.
        sim.Jacobian[6 * Cols + 0] = 1f;
        sim.Jacobian[7 * Cols + 1] = 1f;
        return sim;
    }

    [Fact]
    public void Returns_The_Current_Tcp_Position()
    {
        var sim = Posed();
        var tcp = DiffIk.StepTowards(sim, Robot, TcpLink, 1f, 2f, 3f);
        Assert.Equal((1f, 2f, 3f), tcp);
    }

    [Fact]
    public void Small_Error_Produces_The_Closed_Form_Joint_Deltas()
    {
        var sim = Posed();
        const float gain = 0.5f, damping = 0.05f;

        // Target offset (+0.02, −0.04, 0) → e = offset·gain → dq = e / (1 + λ²).
        DiffIk.StepTowards(sim, Robot, TcpLink, 1.02f, 1.96f, 3f, gain, damping);

        var write = Assert.Single(sim.TargetWrites);
        Assert.Equal(Robot, write.Pattern);
        float denom = 1f + damping * damping;
        Assert.Equal(0.02f * gain / denom, write.Values[0], 1e-5f);
        Assert.Equal(-0.04f * gain / denom, write.Values[1], 1e-5f);
    }

    [Fact]
    public void Large_Error_Is_Clamped_To_Max_Joint_Step()
    {
        var sim = Posed();
        DiffIk.StepTowards(sim, Robot, TcpLink, 2f, 2f, 3f, maxJointStep: 0.05f); // 1 m of x error

        var write = Assert.Single(sim.TargetWrites);
        Assert.Equal(0.05f, write.Values[0], 1e-6f);
        Assert.Equal(0f, write.Values[1], 1e-6f);
    }

    [Fact]
    public void Integrates_Onto_The_Last_Commanded_Target_Not_The_Measurement()
    {
        var sim = Posed();
        sim.DofTargets = [0.30f, -0.10f]; // pre-existing commanded setpoint

        DiffIk.StepTowards(sim, Robot, TcpLink, 1.02f, 1.96f, 3f);

        var write = Assert.Single(sim.TargetWrites);
        float denom = 1f + 0.05f * 0.05f;
        Assert.Equal(0.30f + 0.02f * 0.5f / denom, write.Values[0], 1e-5f);
        Assert.Equal(-0.10f - 0.04f * 0.5f / denom, write.Values[1], 1e-5f);
    }

    [Fact]
    public void Zero_Error_Holds_The_Setpoint_Exactly()
    {
        var sim = Posed();
        sim.DofTargets = [0.42f, -0.17f];

        DiffIk.StepTowards(sim, Robot, TcpLink, 1f, 2f, 3f); // target == current TCP

        var write = Assert.Single(sim.TargetWrites);
        Assert.Equal(0.42f, write.Values[0]);
        Assert.Equal(-0.17f, write.Values[1]);
    }

    [Fact]
    public void Missing_Link_Poses_Return_Null_Without_Writing()
    {
        var sim = new FakeSim { LinkPoses = new float[7], LinkShape = [1, 1, 7] }; // too few links for TcpLink=2

        Assert.Null(DiffIk.StepTowards(sim, Robot, TcpLink, 0f, 0f, 0f));
        Assert.Empty(sim.TargetWrites);
    }

    [Fact]
    public void Missing_Jacobian_Returns_Position_Without_Writing()
    {
        var sim = Posed();
        sim.Jacobian = [];
        sim.JacobianShape = [];

        Assert.Equal((1f, 2f, 3f), DiffIk.StepTowards(sim, Robot, TcpLink, 5f, 5f, 5f));
        Assert.Empty(sim.TargetWrites);
    }
}
