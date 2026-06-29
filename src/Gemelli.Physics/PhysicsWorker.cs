using Nvidia.OvPhysx;
using Gemelli.Core.Ipc;

namespace Gemelli.Physics;

/// <summary>
/// Hosts a single ovphysx instance and serves the <see cref="PhysicsOp"/> command set over a pipe.
/// Lives alone in its own process so ovphysx's Carbonite/Fabric plugins never collide with ovrtx's.
/// </summary>
internal sealed class PhysicsWorker : IDisposable
{
    private PhysX? _physx;
    private TensorBinding? _poseBinding;
    private string[] _primPaths = [];

    // Bindings created on demand and cached, keyed by (tensorType, pattern).
    private readonly Dictionary<(int, string), TensorBinding> _bindings = new();

    /// <summary>
    /// Decodes one <see cref="PhysicsOp"/> from <paramref name="req"/>, mutates ovphysx state and/or
    /// writes the reply to <paramref name="resp"/>. Returns <c>false</c> only for Shutdown to stop the
    /// pipe server; every other op returns <c>true</c> to keep serving.
    /// </summary>
    public bool Dispatch(ushort op, BinaryReader req, BinaryWriter resp)
    {
        switch ((PhysicsOp)op)
        {
            case PhysicsOp.Init:
            {
                // Construct ovphysx on the requested device; must precede every other op.
                var device = (DeviceType)req.ReadInt32();
                _physx = new PhysX(device: device);
                return true;
            }

            case PhysicsOp.LoadUsd:
            {
                // Stage the USD scene; AddUsd is async, so block until the load completes.
                string path = req.ReadString();
                var (_, load) = Physx.AddUsd(path);
                load.Wait(TimeSpan.FromSeconds(120));
                return true;
            }

            case PhysicsOp.BindPoses:
            {
                // (Re)bind the rigid-body pose tensor for the glob and reply with the matched prim paths.
                string pattern = req.ReadString();
                _poseBinding?.Dispose();
                _poseBinding = Physx.CreateTensorBinding(TensorType.RigidBodyPose, pattern, raiseIfEmpty: false);
                _primPaths = _poseBinding.Count == 0 ? [] : _poseBinding.PrimPaths.ToArray();
                Wire.WriteStringArray(resp, _primPaths);
                return true;
            }

            case PhysicsOp.StepAndReadPoses:
            {
                // Advance the sim one tick, then return the count + flat pose buffer for the bridge.
                float dt = req.ReadSingle();
                float simTime = req.ReadSingle();
                Physx.StepSync(dt, simTime);

                if (_primPaths.Length == 0 || _poseBinding is null)
                {
                    resp.Write(0);
                    Wire.WriteFloatArray(resp, ReadOnlySpan<float>.Empty);
                    return true;
                }

                float[] poses = _poseBinding.Read();
                resp.Write(_primPaths.Length);
                Wire.WriteFloatArray(resp, poses);
                return true;
            }

            case PhysicsOp.WriteTensor:
            {
                // Push values into an arbitrary tensor (e.g. DOF targets); raise if the glob matched nothing.
                var tensorType = (TensorType)req.ReadInt32();
                string pattern = req.ReadString();
                float[] values = Wire.ReadFloatArray(req);
                GetBinding(tensorType, pattern, raiseIfEmpty: true).Write(values);
                return true;
            }

            case PhysicsOp.ReadTensor:
            {
                // Read back an arbitrary tensor, returning its shape then the flat values.
                var tensorType = (TensorType)req.ReadInt32();
                string pattern = req.ReadString();
                TensorBinding binding = GetBinding(tensorType, pattern, raiseIfEmpty: false);
                Wire.WriteLongArray(resp, binding.Shape.ToArray());
                Wire.WriteFloatArray(resp, binding.Count == 0 ? [] : binding.Read());
                return true;
            }

            case PhysicsOp.ReadDofNames:
            {
                // Return the ordered DOF names of the matched articulation(s) for client-side labelling.
                string pattern = req.ReadString();
                TensorBinding binding = GetBinding(TensorType.ArticulationDofPosition, pattern, raiseIfEmpty: false);
                Wire.WriteStringArray(resp, binding.Count == 0 ? [] : binding.DofNames.ToArray());
                return true;
            }

            case PhysicsOp.Shutdown:
                return false;

            default:
                throw new InvalidOperationException($"Unknown physics op {op}.");
        }
    }

    /// <summary>The ovphysx instance, guarded so ops that arrive before <see cref="PhysicsOp.Init"/> fail loudly.</summary>
    private PhysX Physx => _physx ?? throw new InvalidOperationException("Physics worker not initialized (send Init first).");

    /// <summary>Returns the tensor binding for (<paramref name="type"/>, <paramref name="pattern"/>), creating and caching it on first use.</summary>
    private TensorBinding GetBinding(TensorType type, string pattern, bool raiseIfEmpty)
    {
        var key = ((int)type, pattern);
        if (!_bindings.TryGetValue(key, out TensorBinding? binding))
        {
            binding = Physx.CreateTensorBinding(type, pattern, raiseIfEmpty);
            _bindings[key] = binding;
        }
        return binding;
    }

    /// <summary>Releases every cached binding, the pose binding, and the ovphysx instance.</summary>
    public void Dispose()
    {
        foreach (TensorBinding b in _bindings.Values) b.Dispose();
        _poseBinding?.Dispose();
        _physx?.Dispose();
    }
}
