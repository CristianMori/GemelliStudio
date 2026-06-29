using System.Numerics;

namespace Gemelli.Core;

/// <summary>
/// Pure, dependency-free conversions between the ovphysx pose convention and the ovrtx
/// transform convention. Kept free of any native types so it is unit-testable without the DLLs.
/// </summary>
/// <remarks>
/// ovphysx <c>RigidBodyPose</c> rows are 7 float32: <c>(px, py, pz, qx, qy, qz, qw)</c> in world
/// space (xyzw, imaginary-first — same component order as <see cref="Quaternion"/>).
/// ovrtx expects <c>omni:xform</c> as a 4×4 float64 matrix in the USD row-vector convention:
/// translation lives in the last row (indices 12,13,14) and the matrix multiplies row vectors
/// (<c>v' = v · M</c>), which is exactly <see cref="Matrix4x4"/>'s memory layout.
/// </remarks>
public static class TransformConversion
{
    /// <summary>Floats per ovphysx pose row.</summary>
    public const int PoseStride = 7;

    /// <summary>Doubles per ovrtx 4×4 matrix.</summary>
    public const int MatrixStride = 16;

    /// <summary>
    /// Converts a single <c>(px,py,pz,qx,qy,qz,qw)</c> pose to a row-major 4×4 double matrix
    /// (USD row-vector layout). <paramref name="pose"/> must have length ≥ 7;
    /// <paramref name="matrix"/> must have length ≥ 16.
    /// </summary>
    public static void PoseToUsdMatrix(ReadOnlySpan<float> pose, Span<double> matrix)
    {
        if (pose.Length < PoseStride)
            throw new ArgumentException($"pose must have at least {PoseStride} elements.", nameof(pose));
        if (matrix.Length < MatrixStride)
            throw new ArgumentException($"matrix must have at least {MatrixStride} elements.", nameof(matrix));

        var q = new Quaternion(pose[3], pose[4], pose[5], pose[6]);
        Matrix4x4 r = Matrix4x4.CreateFromQuaternion(q);

        // Row-major, row-vector convention; rotation 3×3 in the upper-left, translation in the last row.
        matrix[0] = r.M11; matrix[1] = r.M12; matrix[2] = r.M13; matrix[3] = r.M14;
        matrix[4] = r.M21; matrix[5] = r.M22; matrix[6] = r.M23; matrix[7] = r.M24;
        matrix[8] = r.M31; matrix[9] = r.M32; matrix[10] = r.M33; matrix[11] = r.M34;
        matrix[12] = pose[0]; matrix[13] = pose[1]; matrix[14] = pose[2]; matrix[15] = 1.0;
    }

    /// <summary>
    /// Converts a flat <c>[N, 7]</c> pose buffer into a flat <c>[N, 16]</c> matrix buffer.
    /// <paramref name="poses"/> length must be a multiple of 7; the returned buffer is
    /// <c>(N * 16)</c> doubles, ready to hand to ovrtx as a float64×16-lane DLTensor.
    /// </summary>
    public static double[] PosesToUsdMatrices(ReadOnlySpan<float> poses)
    {
        if (poses.Length % PoseStride != 0)
            throw new ArgumentException($"poses length ({poses.Length}) must be a multiple of {PoseStride}.", nameof(poses));

        int n = poses.Length / PoseStride;
        var matrices = new double[n * MatrixStride];
        for (int i = 0; i < n; i++)
        {
            PoseToUsdMatrix(
                poses.Slice(i * PoseStride, PoseStride),
                matrices.AsSpan(i * MatrixStride, MatrixStride));
        }
        return matrices;
    }
}
