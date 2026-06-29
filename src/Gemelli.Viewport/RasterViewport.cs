using System.Numerics;

namespace Gemelli.Viewport;

/// <summary>Camera + target-size snapshot the render thread reads each frame (thread-safe value type).</summary>
public readonly record struct CameraSnapshot(Vector3 Eye, Vector3 Forward, float Vfov, int Width, int Height);

/// <summary>
/// Drives the offscreen <see cref="GlRasterizer"/> on its own thread: loads the scene geometry once, then
/// renders at ~60 fps reading the latest rigid-body poses (so the viewport stays smooth independent of the
/// sim/sensor rate) and the latest camera snapshot. Raises <see cref="FrameReady"/> with a fresh RGBA frame.
/// </summary>
public sealed class RasterViewport : IDisposable
{
    private readonly Thread _thread;
    private volatile bool _stop;

    /// <summary>Raised on the render thread with a freshly-allocated RGBA8 frame (top-left origin).</summary>
    public event Action<byte[], int, int>? FrameReady;

    /// <summary>Raised on the render thread if geometry load / GL setup fails (viewport can fall back to RTX).</summary>
    public event Action<Exception>? Faulted;

    /// <summary>Raised once after geometry loads, with the world-space (center, radius) of the objects — for framing.</summary>
    public event Action<Vector3, float>? Loaded;

    /// <summary>Spins up the background render thread immediately; geometry loads on that thread (see <see cref="Run"/>).</summary>
    public RasterViewport(
        string usdPath,
        IReadOnlyCollection<string> rigidBodyPaths,
        Func<string, float[]?> poseLookup,
        Func<CameraSnapshot> camera)
    {
        _thread = new Thread(() => Run(usdPath, rigidBodyPaths, poseLookup, camera))
        {
            IsBackground = true,
            Name = "gemelli-raster",
        };
        _thread.Start();
    }

    // Render-thread body: load + upload geometry once, then render-loop forever reading the latest camera and
    // per-body pose each frame. All GL lives here so the context stays bound to this one thread.
    private void Run(string usd, IReadOnlyCollection<string> bodies, Func<string, float[]?> pose, Func<CameraSnapshot> camera)
    {
        GlRasterizer? ras = null;
        try
        {
            GeometryResult geo = UsdGeometryLoader.Load(usd, bodies);
            Loaded?.Invoke(geo.Center, geo.Radius);
            CameraSnapshot s0 = camera();
            ras = new GlRasterizer(Math.Max(16, s0.Width), Math.Max(16, s0.Height));
            ras.Upload(geo.Meshes);

            // Fallback model per body = its load-time world transform (inverse of the bake inverse), used
            // when a live physics pose isn't available yet — so geometry sits at its authored pose instead
            // of collapsing to the body-local origin (which looks like floating/scattered parts).
            var loadModel = new Dictionary<string, Matrix4x4>();
            foreach (var (b, binv) in geo.BodyInverse)
                if (Matrix4x4.Invert(binv, out Matrix4x4 load)) loadModel[b] = load;

            // Per-frame model matrix for a body: prefer the live physics pose, fall back to its load-time pose.
            Matrix4x4? Model(string body) =>
                ModelFromPose(pose(body)) ?? (loadModel.TryGetValue(body, out Matrix4x4 m) ? m : null);

            // Uncapped: glReadPixels blocks until each frame is ready, so the loop self-paces at max
            // throughput without busy-spinning. The UI coalesces/drops frames it can't blit in time.
            while (!_stop)
            {
                CameraSnapshot s = camera();
                if (s.Width < 16 || s.Height < 16) { Thread.Sleep(8); continue; }
                ras.Resize(s.Width, s.Height);
                Matrix4x4 view = ViewportCamera.View(s.Eye, s.Forward);
                Matrix4x4 proj = ViewportCamera.Projection(s.Vfov, (float)s.Width / s.Height);
                byte[] frame = ras.Render(view, proj, Model);
                FrameReady?.Invoke((byte[])frame.Clone(), s.Width, s.Height); // clone: rasterizer reuses its buffer
            }
        }
        catch (Exception ex) { Faulted?.Invoke(ex); }
        finally { ras?.Dispose(); }
    }

    // Rigid-body pose (px,py,pz, qx,qy,qz,qw) → row-vector local→world model matrix.
    private static Matrix4x4? ModelFromPose(float[]? pose)
    {
        if (pose is null || pose.Length < 7) return null;
        Matrix4x4 m = Matrix4x4.CreateFromQuaternion(new Quaternion(pose[3], pose[4], pose[5], pose[6]));
        m.M41 = pose[0]; m.M42 = pose[1]; m.M43 = pose[2];
        return m;
    }

    /// <summary>Signals the render loop to exit and waits (bounded) for the thread to unwind and free its GL context.</summary>
    public void Dispose()
    {
        _stop = true;
        _thread.Join(2000);
    }
}
