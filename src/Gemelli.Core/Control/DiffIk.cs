using Gemelli.Core.Ipc;

namespace Gemelli.Core.Control;

/// <summary>
/// Position-only differential inverse kinematics for driving a robot's TCP (tool/end-effector) in
/// Cartesian space. One call performs a single damped-least-squares step toward a world-space target
/// and writes the resulting DOF position targets, so the joint drives move the arm. Call it each
/// frame (e.g. from a script) to track a moving target.
/// </summary>
/// <remarks>
/// Uses ovphysx's articulation Jacobian (<see cref="SimTensor.ArticulationJacobian"/>, shape
/// <c>[N, 6·movingLinks, C]</c>) and link poses (<see cref="SimTensor.ArticulationLinkPose"/>,
/// <c>[N, L, 7]</c>). Assumes a single fixed-base articulation (C == DOF count); the TCP link's linear
/// rows are block <c>(tcpLinkIndex − 1)·6 .. +2</c> because the fixed root link is excluded from the Jacobian.
/// </remarks>
public static class DiffIk
{
    /// <summary>
    /// Steps the arm one IK iteration toward world target (<paramref name="tx"/>,<paramref name="ty"/>,
    /// <paramref name="tz"/>) for link <paramref name="tcpLinkIndex"/> of the articulation matched by
    /// <paramref name="robotPattern"/>. Returns the current TCP position (before the step), or null if
    /// the articulation/Jacobian isn't available.
    /// </summary>
    public static (float X, float Y, float Z)? StepTowards(
        ISimApi sim, string robotPattern, int tcpLinkIndex,
        float tx, float ty, float tz,
        float gain = 0.5f, float damping = 0.05f, float maxJointStep = 0.05f)
    {
        // Current TCP world position from the link-pose tensor [1, L, 7].
        (long[] lshape, float[] link) = sim.ReadShaped(SimTensor.ArticulationLinkPose, robotPattern);
        if (lshape.Length < 2 || link.Length < (tcpLinkIndex + 1) * 7) return null;
        int linkCount = (int)lshape[^2];
        int baseIdx = tcpLinkIndex * 7;
        float px = link[baseIdx], py = link[baseIdx + 1], pz = link[baseIdx + 2];

        // Jacobian [1, R, C]; the TCP's linear rows.
        (long[] jshape, float[] jac) = sim.ReadShaped(SimTensor.ArticulationJacobian, robotPattern);
        if (jshape.Length < 2 || jac.Length == 0) return (px, py, pz);
        int rows = (int)jshape[^2], cols = (int)jshape[^1];
        int movingLinks = rows / 6;
        int block = movingLinks == linkCount ? tcpLinkIndex : tcpLinkIndex - 1; // fixed base drops root link0
        if (block < 0 || (block + 1) * 6 > rows) return (px, py, pz);
        int row0 = block * 6; // linear rows row0..row0+2

        // Position error (chase a fraction per call via gain).
        float ex = (tx - px) * gain, ey = (ty - py) * gain, ez = (tz - pz) * gain;

        // J = 3×cols (linear part). A = J Jᵀ + λ²I  (3×3). dq = Jᵀ A⁻¹ e.
        Span<float> A = stackalloc float[9];
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
            {
                float s = 0;
                for (int k = 0; k < cols; k++) s += jac[(row0 + r) * cols + k] * jac[(row0 + c) * cols + k];
                A[r * 3 + c] = s + (r == c ? damping * damping : 0);
            }

        if (!Invert3x3(A, out Span<float> Ainv)) return (px, py, pz);

        // y = A⁻¹ e
        float yx = Ainv[0] * ex + Ainv[1] * ey + Ainv[2] * ez;
        float yy = Ainv[3] * ex + Ainv[4] * ey + Ainv[5] * ez;
        float yz = Ainv[6] * ex + Ainv[7] * ey + Ainv[8] * ez;

        // dq = Jᵀ y, then q += dq (clamped), and command position targets.
        // Integrate onto the last COMMANDED target, not the measured DOF position. Joint drives have finite
        // stiffness, so the measured position always sags slightly below the commanded target under gravity;
        // re-reading the measurement each frame and re-commanding it would ratchet the target downward every
        // frame (a slow downward drift even with zero input). Reading the target view holds the setpoint fixed
        // when the error is zero. Fall back to the measured position if the target view is unreadable.
        float[] q = sim.Read(SimTensor.ArticulationDofPositionTarget, robotPattern);
        if (q.Length < cols) q = sim.Read(SimTensor.ArticulationDofPosition, robotPattern);
        int n = Math.Min(q.Length, cols);
        for (int j = 0; j < n; j++)
        {
            float dq = jac[row0 * cols + j] * yx + jac[(row0 + 1) * cols + j] * yy + jac[(row0 + 2) * cols + j] * yz;
            q[j] += Math.Clamp(dq, -maxJointStep, maxJointStep);
        }
        sim.SetDofPositionTargets(robotPattern, q);
        return (px, py, pz);
    }

    // Inverts a 3×3 row-major matrix via cofactors; returns false (singular) if the determinant is ~0.
    private static bool Invert3x3(ReadOnlySpan<float> m, out Span<float> inv)
    {
        inv = new float[9];
        float a = m[0], b = m[1], c = m[2], d = m[3], e = m[4], f = m[5], g = m[6], h = m[7], i = m[8];
        float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        if (MathF.Abs(det) < 1e-9f) return false;
        float id = 1f / det;
        inv[0] = (e * i - f * h) * id; inv[1] = (c * h - b * i) * id; inv[2] = (b * f - c * e) * id;
        inv[3] = (f * g - d * i) * id; inv[4] = (a * i - c * g) * id; inv[5] = (c * d - a * f) * id;
        inv[6] = (d * h - e * g) * id; inv[7] = (b * g - a * h) * id; inv[8] = (a * e - b * d) * id;
        return true;
    }
}
