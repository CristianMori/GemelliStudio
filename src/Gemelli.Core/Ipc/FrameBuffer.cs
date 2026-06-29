using System.IO.MemoryMappedFiles;
using System.Runtime.Versioning;

namespace Gemelli.Core.Ipc;

/// <summary>
/// A cross-process shared-memory region for the rendered framebuffer. The render worker writes pixel
/// data here and sends only small metadata (offsets/shapes) over the pipe, avoiding serialization of
/// the multi-MB image through the named pipe each frame. Backed by a named page-file mapping
/// (Windows); the request/reply pipe round-trip provides the read/write synchronization (the worker
/// writes before replying; the host reads after the reply, before the next render request).
/// </summary>
[SupportedOSPlatform("windows")]
public sealed unsafe class FrameBuffer : IDisposable
{
    private readonly MemoryMappedFile _mmf;
    private readonly MemoryMappedViewAccessor _view;
    private byte* _ptr;
    private bool _disposed;

    public string Name { get; }
    public long Capacity { get; }

    /// <summary>Maps the full region and pins a raw pointer to its base for direct span copies (the
    /// pointer is released in <see cref="Dispose"/>).</summary>
    private FrameBuffer(string name, long capacity, MemoryMappedFile mmf)
    {
        Name = name;
        Capacity = capacity;
        _mmf = mmf;
        _view = _mmf.CreateViewAccessor(0, capacity, MemoryMappedFileAccess.ReadWrite);
        _view.SafeMemoryMappedViewHandle.AcquirePointer(ref _ptr);
    }

    /// <summary>Host side: create a new named region of the given capacity.</summary>
    public static FrameBuffer Create(string name, long capacity) =>
        new(name, capacity, MemoryMappedFile.CreateNew(name, capacity, MemoryMappedFileAccess.ReadWrite));

    /// <summary>Worker side: open an existing named region created by the host.</summary>
    public static FrameBuffer Open(string name, long capacity) =>
        new(name, capacity, MemoryMappedFile.OpenExisting(name, MemoryMappedFileRights.ReadWrite));

    /// <summary>Copies <paramref name="source"/> into the region at <paramref name="offset"/>.</summary>
    public void Write(long offset, ReadOnlySpan<byte> source)
    {
        if (offset < 0 || offset + source.Length > Capacity)
            throw new ArgumentOutOfRangeException(nameof(offset), "Frame write exceeds shared-buffer capacity.");
        source.CopyTo(new Span<byte>(_ptr + offset, source.Length));
    }

    /// <summary>Copies <paramref name="length"/> bytes from <paramref name="offset"/> into a new managed array.</summary>
    public byte[] Read(long offset, int length)
    {
        if (offset < 0 || offset + length > Capacity)
            throw new ArgumentOutOfRangeException(nameof(offset), "Frame read exceeds shared-buffer capacity.");
        var dst = new byte[length];
        new ReadOnlySpan<byte>(_ptr + offset, length).CopyTo(dst);
        return dst;
    }

    /// <summary>Raw destination pointer for a direct native→shared copy (worker side).</summary>
    public byte* Pointer => _ptr;

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        if (_ptr != null) { _view.SafeMemoryMappedViewHandle.ReleasePointer(); _ptr = null; }
        _view.Dispose();
        _mmf.Dispose();
    }
}
