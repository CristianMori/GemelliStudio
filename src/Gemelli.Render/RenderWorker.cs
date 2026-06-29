using Nvidia.Ovrtx;
using Nvidia.Ovrtx.Interop;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Render;

/// <summary>
/// Hosts a single ovrtx renderer and serves the <see cref="RenderOp"/> command set over a pipe.
/// Lives alone in its own process so ovrtx's plugins never collide with ovphysx's. Receives
/// rigid-body transforms (already converted to USD 4×4 matrices) and writes them to <c>omni:xform</c>
/// before each render.
/// </summary>
internal sealed unsafe class RenderWorker : IDisposable
{
    private const string XformAttribute = "omni:xform";
    private Renderer? _renderer;
    private FrameBuffer? _frameBuffer;
    private bool _renderEnabled = true;

    /// <summary>
    /// Decodes one <see cref="RenderOp"/> from <paramref name="req"/>, drives ovrtx, and writes the reply
    /// to <paramref name="resp"/>. Returns <c>false</c> only for Shutdown to stop the pipe server.
    /// </summary>
    public bool Dispatch(ushort op, BinaryReader req, BinaryWriter resp)
    {
        switch ((RenderOp)op)
        {
            case RenderOp.Init:
            {
                // Configure native lib path + optional shared frame buffer, then construct the renderer
                // and reply with its version triple.
                string ovrtxDir = req.ReadString();
                bool syncMode = req.ReadBoolean();
                string frameBufferName = req.ReadString();
                long frameBufferCapacity = req.ReadInt64();
                _renderEnabled = req.ReadBoolean();
                if (!string.IsNullOrEmpty(ovrtxDir))
                    OvrtxLibrary.SetLibraryPath(ovrtxDir);
                if (!string.IsNullOrEmpty(frameBufferName))
                    _frameBuffer = FrameBuffer.Open(frameBufferName, frameBufferCapacity);
                _renderer = new Renderer(new RendererConfig { SyncMode = syncMode });
                var (maj, min, patch) = Renderer.Version;
                resp.Write(maj); resp.Write(min); resp.Write(patch);
                return true;
            }

            case RenderOp.LoadUsd:
            {
                // Stage the same USD scene the physics worker loaded, this time for rendering.
                string path = req.ReadString();
                Render.AddUsd(path);
                return true;
            }

            case RenderOp.Warmup:
            {
                // Render-and-discard a few frames so the first measured frame isn't paying shader compilation.
                string[] products = Wire.ReadStringArray(req);
                int frames = req.ReadInt32();
                if (_renderEnabled)
                    for (int i = 0; i < frames; i++)
                    {
                        using RenderProductSetOutputs outputs = Render.Step(products, 0.0);
                    }
                return true;
            }

            case RenderOp.WriteAndStep:
            {
                // The per-frame hot path: apply the latest physics transforms, render, return frames.
                string[] paths = Wire.ReadStringArray(req);
                double[] matrices = Wire.ReadDoubleArray(req);
                string[] products = Wire.ReadStringArray(req);
                double dt = req.ReadDouble();

                if (paths.Length > 0)
                    WriteTransforms(paths, matrices);

                // Measurement mode: run the full bridge but skip the ovrtx render; return no frames.
                if (!_renderEnabled)
                {
                    resp.Write(false);                 // inline path
                    FrameCodec.Write(resp, new List<CapturedFrame>());
                    return true;
                }

                using RenderProductSetOutputs outputs = Render.Step(products, dt);
                if (_frameBuffer is not null)
                {
                    // Pixels go to shared memory; only offsets/shapes go over the pipe.
                    List<FrameLayout> layout = SensorHub.ExtractToBuffer(outputs, _frameBuffer);
                    resp.Write(true); // shared-memory path
                    FrameLayoutCodec.Write(resp, layout);
                }
                else
                {
                    List<CapturedFrame> frames = SensorHub.Extract(outputs);
                    resp.Write(false); // inline path
                    FrameCodec.Write(resp, frames);
                }
                return true;
            }

            case RenderOp.Shutdown:
                return false;

            default:
                throw new InvalidOperationException($"Unknown render op {op}.");
        }
    }

    /// <summary>The ovrtx instance, guarded so ops that arrive before <see cref="RenderOp.Init"/> fail loudly.</summary>
    private Renderer Render => _renderer ?? throw new InvalidOperationException("Render worker not initialized (send Init first).");

    /// <summary>
    /// Writes <paramref name="paths"/>'s world transforms into <c>omni:xform</c> as one DLTensor of
    /// 4×4 float64 matrices (16 lanes each), so the renderer reflects the latest physics poses.
    /// </summary>
    private void WriteTransforms(string[] paths, double[] matrices)
    {
        // One DLTensor of N elements, each a 4×4 float64 matrix packed as 16 lanes -> omni:xform.
        var dtype = new DLDataType(DLDataTypeCode.Float, bits: 64, lanes: 16);
        fixed (double* data = matrices)
        {
            long shape = paths.Length;
            var tensor = new DLTensor
            {
                Data = (IntPtr)data,
                Device = DLDevice.Cpu,
                NDim = 1,
                DType = dtype,
                Shape = (IntPtr)(&shape),
                Strides = IntPtr.Zero,
                ByteOffset = 0,
            };

            Render.WriteAttribute(
                paths, XformAttribute, &tensor,
                semantic: Semantic.XformMat4x4,
                primMode: PrimMode.ExistingOnly,
                dataAccess: DataAccess.Sync);
        }
    }

    /// <summary>Releases the shared frame buffer and the ovrtx instance.</summary>
    public void Dispose()
    {
        _frameBuffer?.Dispose();
        _renderer?.Dispose();
    }
}
