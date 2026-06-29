using Nvidia.Ovrtx;
using Nvidia.Ovrtx.Interop;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;

namespace Gemelli.Render;

/// <summary>
/// Drains an ovrtx <see cref="RenderProductSetOutputs"/> into either managed <see cref="CapturedFrame"/>s
/// (pixels copied for the pipe) or a shared <see cref="FrameBuffer"/> (pixels written to shared memory,
/// only offsets returned). Each render variable is mapped to CPU, copied out, then unmapped.
/// </summary>
internal static class SensorHub
{
    /// <summary>
    /// Copies every render variable of every frame into managed <see cref="CapturedFrame"/>s for the
    /// inline pipe path (no shared buffer configured). Pixels are duplicated into managed byte arrays.
    /// </summary>
    public static List<CapturedFrame> Extract(RenderProductSetOutputs outputs)
    {
        var frames = new List<CapturedFrame>();
        foreach (var (productPath, product) in outputs.Products)
        {
            foreach (FrameOutput frame in product.Frames)
            {
                var vars = new Dictionary<string, RenderVarData>(frame.RenderVars.Count);
                foreach (var (varName, varOutput) in frame.RenderVars)
                {
                    // Map native pixels to CPU, copy them out, then unmap (the using scope).
                    using MappedRenderVar mapped = varOutput.Map(MapDeviceType.Cpu);
                    vars[varName] = CopyTensor(varName, mapped.Tensor);
                }
                frames.Add(new CapturedFrame(productPath, frame.StartTime, frame.EndTime, vars));
            }
        }
        return frames;
    }

    /// <summary>
    /// Writes each render variable's pixels directly into <paramref name="buffer"/> (native→shared,
    /// one copy) and returns layout metadata describing where each lives. Avoids serializing pixels
    /// through the pipe. Throws if a frame exceeds the shared buffer capacity.
    /// </summary>
    public static unsafe List<FrameLayout> ExtractToBuffer(RenderProductSetOutputs outputs, FrameBuffer buffer)
    {
        var frames = new List<FrameLayout>();
        long offset = 0;
        foreach (var (productPath, product) in outputs.Products)
        {
            foreach (FrameOutput frame in product.Frames)
            {
                var vars = new List<VarLayout>(frame.RenderVars.Count);
                foreach (var (varName, varOutput) in frame.RenderVars)
                {
                    using MappedRenderVar mapped = varOutput.Map(MapDeviceType.Cpu);
                    DLTensor t = mapped.Tensor;
                    long[] shape = t.GetShape().ToArray();
                    long elements = 1;
                    foreach (long d in shape) elements *= d;
                    DLDataType dtype = t.DType;
                    int bpe = Math.Max(1, dtype.Bits / 8) * Math.Max((ushort)1, dtype.Lanes);
                    int byteCount = checked((int)(elements * bpe));

                    if (t.Data != IntPtr.Zero && byteCount > 0)
                    {
                        var src = new ReadOnlySpan<byte>((void*)(t.Data + (nint)t.ByteOffset), byteCount);
                        buffer.Write(offset, src); // native → shared memory (single copy)
                    }
                    vars.Add(new VarLayout(varName, shape, MapType(dtype.Code), dtype.Bits, dtype.Lanes, offset, byteCount));
                    offset += byteCount;
                }
                frames.Add(new FrameLayout(productPath, frame.StartTime, frame.EndTime, vars));
            }
        }
        return frames;
    }

    /// <summary>
    /// Copies a single mapped DLTensor's raw bytes into a managed <see cref="RenderVarData"/>, deriving
    /// element count and bytes-per-element from its shape and dtype (bits × lanes).
    /// </summary>
    private static unsafe RenderVarData CopyTensor(string name, DLTensor tensor)
    {
        long[] shape = tensor.GetShape().ToArray();
        long elements = 1;
        foreach (long d in shape) elements *= d;

        DLDataType dtype = tensor.DType;
        int bytesPerElement = Math.Max(1, dtype.Bits / 8) * Math.Max((ushort)1, dtype.Lanes);
        long byteCount = elements * bytesPerElement;

        var bytes = new byte[byteCount];
        if (tensor.Data != IntPtr.Zero && byteCount > 0)
        {
            var src = new ReadOnlySpan<byte>((void*)(tensor.Data + (nint)tensor.ByteOffset), checked((int)byteCount));
            src.CopyTo(bytes);
        }

        return new RenderVarData(name, shape, MapType(dtype.Code), dtype.Bits, dtype.Lanes, bytes);
    }

    /// <summary>Maps a DLPack dtype code to the wire <see cref="ScalarType"/>; unknown codes become Other.</summary>
    private static ScalarType MapType(DLDataTypeCode code) => code switch
    {
        DLDataTypeCode.Int => ScalarType.Int,
        DLDataTypeCode.UInt => ScalarType.UInt,
        DLDataTypeCode.Float => ScalarType.Float,
        DLDataTypeCode.Bool => ScalarType.Bool,
        _ => ScalarType.Other,
    };
}
