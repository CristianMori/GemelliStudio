using System.Numerics;

namespace Gemelli.Studio;

/// <summary>
/// A turntable camera orbiting a target in spherical coordinates (azimuth, elevation, distance).
/// Produces a USD row-vector 4×4 camera-to-world matrix (16 doubles) — the camera looks down its
/// local −Z, with +Y up and +X right. Z-up world. Driven by mouse: drag to orbit, drag+shift to pan,
/// wheel to zoom.
/// </summary>
public sealed class OrbitCamera
{
    private float _azimuth = 2.4f;     // radians around the up axis
    private float _elevation = 0.5f;   // radians above the horizon
    private float _distance = 32f;
    private Vector3 _target = new(10f, -1f, 4f); // framed on the falling-boxes row by default

    /// <summary>Drag-orbit: advances azimuth/elevation, clamping elevation short of the poles.</summary>
    public void Orbit(float dAzimuth, float dElevation)
    {
        _azimuth += dAzimuth;
        _elevation = Math.Clamp(_elevation + dElevation, -1.5f, 1.5f);
    }

    /// <summary>Wheel-zoom: scales orbit distance by <paramref name="factor"/>, clamped to a sane range.</summary>
    public void Zoom(float factor) => _distance = Math.Clamp(_distance * factor, 0.5f, 5000f);

    public void Pan(float dx, float dy)
    {
        // Pan in the camera's screen plane, scaled by distance for consistent feel.
        Basis(out Vector3 right, out Vector3 up, out _, out _);
        float s = _distance * 0.0015f;
        _target += right * (-dx * s) + up * (dy * s);
    }

    /// <summary>Frames the camera on a target at a given distance.</summary>
    public void Frame(Vector3 target, float distance)
    {
        _target = target;
        _distance = Math.Clamp(distance, 0.5f, 5000f);
    }

    /// <summary>Frames on a target (component form, so callers needn't reference System.Numerics).</summary>
    public void Frame(float x, float y, float z, float distance) => Frame(new Vector3(x, y, z), distance);

    /// <summary>Current eye position and orthonormal view basis (right, up, forward=look direction).</summary>
    public void GetView(out Vector3 eye, out Vector3 right, out Vector3 up, out Vector3 forward)
    {
        Basis(out Vector3 x, out Vector3 y, out Vector3 z, out eye);
        right = x; up = y; forward = -z; // camera looks down −Z
    }

    /// <summary>The current camera-to-world transform as a USD row-major 4×4 (16 doubles).</summary>
    public double[] ToUsdMatrix()
    {
        Basis(out Vector3 x, out Vector3 y, out Vector3 z, out Vector3 eye);
        return
        [
            x.X, x.Y, x.Z, 0,
            y.X, y.Y, y.Z, 0,
            z.X, z.Y, z.Z, 0,
            eye.X, eye.Y, eye.Z, 1,
        ];
    }

    /// <summary>Computes the eye position and orthonormal camera basis (x=right, y=up, z=back) from the orbit state.</summary>
    private void Basis(out Vector3 x, out Vector3 y, out Vector3 z, out Vector3 eye)
    {
        // Z-up spherical position around the target.
        float ce = MathF.Cos(_elevation), se = MathF.Sin(_elevation);
        var offset = new Vector3(MathF.Cos(_azimuth) * ce, MathF.Sin(_azimuth) * ce, se) * _distance;
        eye = _target + offset;

        var worldUp = new Vector3(0, 0, 1);
        z = Vector3.Normalize(eye - _target);              // camera +Z points back toward the eye
        x = Vector3.Normalize(Vector3.Cross(worldUp, z));
        if (!float.IsFinite(x.X)) x = new Vector3(1, 0, 0); // guard near the pole
        y = Vector3.Cross(z, x);
    }
}
