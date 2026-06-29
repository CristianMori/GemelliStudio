using System.Numerics;

namespace Gemelli.Viewport;

/// <summary>
/// Builds OpenGL-correct view + projection matrices in System.Numerics (row-vector) convention, matching
/// the rest of the pipeline (the rasterizer uploads these transposed so GLSL's column-vector math agrees).
/// Z-up world, right-handed (camera looks down its −forward like USD/GL).
/// </summary>
public static class ViewportCamera
{
    /// <summary>Right-handed look-at view matrix (Z-up world).</summary>
    public static Matrix4x4 View(Vector3 eye, Vector3 forward) =>
        Matrix4x4.CreateLookAt(eye, eye + forward, new Vector3(0, 0, 1));

    /// <summary>
    /// OpenGL perspective (clip z in [−1,1]) in row-vector layout: <c>clip = viewPos · Proj</c>. This is
    /// the transpose of the textbook column-major GL projection.
    /// </summary>
    public static Matrix4x4 Projection(float vfovRadians, float aspect, float near = 0.02f, float far = 1000f)
    {
        float f = 1f / MathF.Tan(vfovRadians * 0.5f);
        var m = new Matrix4x4();
        m.M11 = f / aspect;                     // x scale (cotangent of half-fov, corrected for aspect)
        m.M22 = f;                              // y scale
        m.M33 = (far + near) / (near - far);    // z scale + M43 bias map [near,far] → NDC z [−1,1]
        m.M34 = -1f;                            // pushes −viewZ into clip.w (perspective divide)
        m.M43 = (2f * far * near) / (near - far);
        return m;
    }
}
