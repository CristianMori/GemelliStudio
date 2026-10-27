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

    /// <summary>
    /// Converts a flat <c>[N, 7]</c> world-pose buffer into a flat <c>[N, 16]</c> matrix buffer,
    /// re-expressing each pose relative to its nearest bridged ancestor (per
    /// <paramref name="ancestorIndex"/>, -1 for none). The renderer composes <c>omni:xform</c>
    /// down the prim hierarchy, so a body nested under another bridged body must be written
    /// parent-relative or it is double-transformed (v' = v * M row-vector convention:
    /// local = world_child * inverse(world_ancestor)).
    /// </summary>
    public static double[] PosesToUsdMatrices(ReadOnlySpan<float> poses, ReadOnlySpan<int> ancestorIndex)
    {
        if (poses.Length % PoseStride != 0)
            throw new ArgumentException($"poses length ({poses.Length}) must be a multiple of {PoseStride}.", nameof(poses));
        int n = poses.Length / PoseStride;
        if (ancestorIndex.Length != n)
            throw new ArgumentException($"ancestorIndex length ({ancestorIndex.Length}) must match pose count ({n}).", nameof(ancestorIndex));

        var matrices = new double[n * MatrixStride];
        for (int i = 0; i < n; i++)
        {
            Span<double> dst = matrices.AsSpan(i * MatrixStride, MatrixStride);
            int a = ancestorIndex[i];
            if (a < 0)
            {
                PoseToUsdMatrix(poses.Slice(i * PoseStride, PoseStride), dst);
                continue;
            }
            Matrix4x4 child = PoseToMatrix(poses.Slice(i * PoseStride, PoseStride));
            Matrix4x4 ancestor = PoseToMatrix(poses.Slice(a * PoseStride, PoseStride));
            Matrix4x4.Invert(ancestor, out Matrix4x4 inv);
            Matrix4x4 local = child * inv; // row-vector: world_child = local * world_ancestor
            dst[0] = local.M11; dst[1] = local.M12; dst[2] = local.M13; dst[3] = local.M14;
            dst[4] = local.M21; dst[5] = local.M22; dst[6] = local.M23; dst[7] = local.M24;
            dst[8] = local.M31; dst[9] = local.M32; dst[10] = local.M33; dst[11] = local.M34;
            dst[12] = local.M41; dst[13] = local.M42; dst[14] = local.M43; dst[15] = 1.0;
        }
        return matrices;
    }

    /// <summary>Builds the row-vector world matrix for one <c>(px,py,pz,qx,qy,qz,qw)</c> pose.</summary>
    private static Matrix4x4 PoseToMatrix(ReadOnlySpan<float> pose)
    {
        Matrix4x4 m = Matrix4x4.CreateFromQuaternion(new Quaternion(pose[3], pose[4], pose[5], pose[6]));
        m.M41 = pose[0]; m.M42 = pose[1]; m.M43 = pose[2];
        return m;
    }

    /// <summary>
    /// For each prim path, the index of its nearest ancestor that is itself in
    /// <paramref name="primPaths"/>, or -1. Ancestry is by USD path prefix (segment-aligned).
    /// </summary>
    public static int[] NearestBridgedAncestors(IReadOnlyList<string> primPaths)
    {
        var index = new Dictionary<string, int>(primPaths.Count);
        for (int i = 0; i < primPaths.Count; i++) index[primPaths[i]] = i;

        var result = new int[primPaths.Count];
        for (int i = 0; i < primPaths.Count; i++)
        {
            result[i] = -1;
            string path = primPaths[i];
            for (int cut = path.LastIndexOf('/'); cut > 0; cut = path.LastIndexOf('/', cut - 1))
            {
                if (index.TryGetValue(path[..cut], out int a)) { result[i] = a; break; }
            }
        }
        return result;
    }
}
