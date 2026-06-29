using System.IO.Pipes;

namespace Gemelli.Core.Ipc;

/// <summary>
/// Orchestrator-side request/reply client over a named pipe. Each <see cref="Request"/> writes an
/// opcode + args frame and blocks for the reply frame; a non-zero status byte surfaces as a
/// <see cref="TwinWorkerException"/>. Not thread-safe — one outstanding request at a time.
/// </summary>
public sealed class PipeClient : IDisposable
{
    private readonly NamedPipeClientStream _pipe;

    /// <summary>Connects to the worker's named pipe on the local machine, blocking up to
    /// <paramref name="connectTimeoutMs"/> for the worker to create its end.</summary>
    public PipeClient(string pipeName, int connectTimeoutMs = 30_000)
    {
        _pipe = new NamedPipeClientStream(".", pipeName, PipeDirection.InOut);
        _pipe.Connect(connectTimeoutMs);
    }

    /// <summary>
    /// Sends a request and returns a reader positioned at the start of the reply payload (after the
    /// status byte). <paramref name="writeArgs"/> serializes the request arguments.
    /// </summary>
    public BinaryReader Request(ushort op, Action<BinaryWriter>? writeArgs = null)
    {
        using var reqStream = new MemoryStream();
        using (var reqWriter = new BinaryWriter(reqStream, System.Text.Encoding.UTF8, leaveOpen: true))
        {
            reqWriter.Write(op);
            writeArgs?.Invoke(reqWriter);
        }
        Frame.Write(_pipe, reqStream.GetBuffer().AsSpan(0, (int)reqStream.Length));

        // A null read means the worker closed the pipe without replying — a crash/disconnect, distinct
        // from a worker-reported error (status byte). WorkerHost enriches this with process diagnostics.
        byte[] response = Frame.Read(_pipe)
            ?? throw new TwinWorkerDisconnectedException("Worker closed the pipe without replying.");

        var respStream = new MemoryStream(response, writable: false);
        var respReader = new BinaryReader(respStream);
        byte status = respReader.ReadByte();
        if (status != 0)
        {
            string message = respReader.ReadString();
            respReader.Dispose();
            throw new TwinWorkerException(message);
        }
        return respReader;
    }

    /// <summary>Convenience for requests whose reply carries no payload.</summary>
    public void Send(ushort op, Action<BinaryWriter>? writeArgs = null) => Request(op, writeArgs).Dispose();

    public void Dispose() => _pipe.Dispose();
}
