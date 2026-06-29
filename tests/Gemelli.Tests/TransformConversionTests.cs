using System.Numerics;
using Gemelli.Core;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests: pure pose→matrix conversion, no native libraries required.
/// </summary>
public class TransformConversionTests
{
    // Identity pose (zero translation, unit quaternion) maps to the identity matrix.
    [Fact]
    public void Identity_Pose_Produces_Identity_Matrix()
    {
        // (px,py,pz, qx,qy,qz,qw) — identity rotation is (0,0,0,1).
        float[] pose = [0, 0, 0, 0, 0, 0, 1];
        var m = new double[16];

        TransformConversion.PoseToUsdMatrix(pose, m);

        double[] expected = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
        for (int i = 0; i < 16; i++)
            Assert.Equal(expected[i], m[i], precision: 6);
    }

    // Translation is written to the last row (indices 12-14), per USD's row-vector convention.
    [Fact]
    public void Translation_Lands_In_Last_Row_USD_Convention()
    {
        float[] pose = [5, 6, 7, 0, 0, 0, 1];
        var m = new double[16];

        TransformConversion.PoseToUsdMatrix(pose, m);

        // USD row-vector layout: translation at indices 12,13,14.
        Assert.Equal(5, m[12], 6);
        Assert.Equal(6, m[13], 6);
        Assert.Equal(7, m[14], 6);
        Assert.Equal(1, m[15], 6);
    }

    // Rotation block matches System.Numerics' row-vector matrix for the same quaternion.
    [Fact]
    public void Rotation_Matches_SystemNumerics_RowVector_Layout()
    {
        // 90° about Z.
        Quaternion q = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, MathF.PI / 2f);
        float[] pose = [1, 2, 3, q.X, q.Y, q.Z, q.W];
        var m = new double[16];

        TransformConversion.PoseToUsdMatrix(pose, m);

        Matrix4x4 r = Matrix4x4.CreateFromQuaternion(q);
        Assert.Equal(r.M11, m[0], 5);
        Assert.Equal(r.M12, m[1], 5);
        Assert.Equal(r.M21, m[4], 5);
        Assert.Equal(r.M22, m[5], 5);
        Assert.Equal(r.M33, m[10], 5);
    }

    // Batch path emits a contiguous 16-double matrix per input pose, in order.
    [Fact]
    public void Batch_Conversion_Produces_16_Doubles_Per_Pose()
    {
        float[] poses =
        [
            0, 0, 0, 0, 0, 0, 1,
            1, 1, 1, 0, 0, 0, 1,
        ];

        double[] matrices = TransformConversion.PosesToUsdMatrices(poses);

        Assert.Equal(32, matrices.Length);
        Assert.Equal(0, matrices[12], 6);   // first pose translation x
        Assert.Equal(1, matrices[16 + 12], 6); // second pose translation x
    }

    // A pose buffer whose length isn't a multiple of 7 is rejected.
    [Fact]
    public void Mismatched_Pose_Length_Throws()
    {
        Assert.Throws<ArgumentException>(() => TransformConversion.PosesToUsdMatrices(new float[] { 1, 2, 3 }));
    }
}
