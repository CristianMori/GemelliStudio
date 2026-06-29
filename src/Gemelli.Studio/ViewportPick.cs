using System.Numerics;

namespace Gemelli.Studio;

/// <summary>
/// Screen↔world math for picking and dragging objects against the 2D rendered viewport. Uses the orbit
/// camera's basis and a pinhole projection matching the render camera (vertical FOV derived from focal
/// length + horizontal aperture under USD's default horizontal-aperture fit). No 3D engine needed.
/// </summary>
internal static class ViewportPick
{
    /// <summary>Vertical field of view (radians) for an image of the given aspect (w/h), under horizontal fit.</summary>
    public static float VerticalFov(float focalLengthMm, float horizontalApertureMm, float aspect)
    {
        float horizFov = 2f * MathF.Atan(horizontalApertureMm / (2f * focalLengthMm));
        return 2f * MathF.Atan(MathF.Tan(horizFov * 0.5f) / MathF.Max(aspect, 1e-3f));
    }

    /// <summary>
    /// Projects a world point to image pixel coordinates (origin top-left). Returns false if the point is
    /// behind the camera. <paramref name="dist"/> is the distance along the view direction (for depth tests).
    /// </summary>
    public static bool ProjectToImage(
        Vector3 world, OrbitCamera cam, float vfov, int imgW, int imgH,
        out float px, out float py, out float dist)
    {
        cam.GetView(out Vector3 eye, out Vector3 right, out Vector3 up, out Vector3 fwd);
        Vector3 rel = world - eye;
        float z = Vector3.Dot(rel, fwd); // depth in front of camera
        px = py = 0; dist = z;
        if (z <= 1e-4f) return false;

        float x = Vector3.Dot(rel, right);
        float y = Vector3.Dot(rel, up);
        float aspect = (float)imgW / MathF.Max(1, imgH);
        float tanV = MathF.Tan(vfov * 0.5f);
        float ndcX = x / (z * tanV * aspect);
        float ndcY = y / (z * tanV);
        px = (ndcX * 0.5f + 0.5f) * imgW;
        py = (0.5f - ndcY * 0.5f) * imgH; // NDC +Y is up; image +Y is down
        return true;
    }

    /// <summary>
    /// World-space translation for a screen drag (in image pixels) of an object held at view-depth
    /// <paramref name="dist"/>, on the camera-facing plane. Maps cursor motion 1:1 onto that plane.
    /// </summary>
    public static Vector3 DragDeltaWorld(
        float dPxX, float dPxY, OrbitCamera cam, float vfov, int imgW, int imgH, float dist)
    {
        cam.GetView(out _, out Vector3 right, out Vector3 up, out Vector3 fwd);
        _ = fwd;
        float tanV = MathF.Tan(vfov * 0.5f);
        float worldPerPxY = (2f * dist * tanV) / MathF.Max(1, imgH);
        float worldPerPxX = worldPerPxY; // square pixels: same world/px in both axes
        return right * (dPxX * worldPerPxX) + up * (-dPxY * worldPerPxY); // screen +Y down → world up is −dPxY
    }

    /// <summary>
    /// Intersects the eye→pixel ray with the horizontal plane Z = <paramref name="planeZ"/> and returns the
    /// world hit point — i.e. the floor point under the cursor at a given height. Null if the ray is
    /// parallel to / points away from the plane (grazing camera angle).
    /// </summary>
    public static Vector3? GroundPoint(OrbitCamera cam, float vfov, int imgW, int imgH, float pxX, float pxY, float planeZ)
    {
        cam.GetView(out Vector3 eye, out Vector3 right, out Vector3 up, out Vector3 fwd);
        float aspect = (float)imgW / MathF.Max(1, imgH);
        float tanV = MathF.Tan(vfov * 0.5f);
        float ndcX = pxX / imgW * 2f - 1f;
        float ndcY = 1f - pxY / imgH * 2f;
        Vector3 dir = Vector3.Normalize(fwd + right * (ndcX * tanV * aspect) + up * (ndcY * tanV));
        if (MathF.Abs(dir.Z) < 1e-5f) return null;
        float t = (planeZ - eye.Z) / dir.Z;
        if (t <= 0) return null;
        return eye + dir * t;
    }
}
