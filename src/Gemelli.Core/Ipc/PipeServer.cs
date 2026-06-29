using System.IO.Pipes;

namespace Gemelli.Core.Ipc;

/// <summary>
/// Worker-side request/reply server over a single named pipe. Reads request frames, dispatches by
/// opcode to a handler that writes the reply payload, and frames each reply with a leading status
/// byte (0 = ok, 1 = error + message). One connection, one thread — matching the single-threaded
/// nature of both native runtimes.
/// </summary>
public sealed class PipeServer
{
    private readonly string _pipeName;

    public PipeServer(string pipeName) => _pipeName = pipeName;

    /// <summary>
    /// Accepts a connection and serves requests until the dispatcher returns <c>false</c> (e.g. on
    /// Shutdown) or the client disconnects. <paramref name="dispatch"/> reads args from the request
    /// reader and writes results to the reply writer; returning <c>false</c> stops the loop.
    /// </summary>
    public void Run(Func<ushort, BinaryReader, BinaryWriter, bool> dispatch)
    {
        using var pipe = new NamedPipeServerStream(
            _pipeName, PipeDirection.InOut, 1, PipeTransmissionMode.Byte, PipeOptions.None,
            inBufferSize: 0, outBufferSize: 0);

        pipe.WaitForConnection();

        bool keepRunning = true;
        while (keepRunning && pipe.IsConnected)
        {
            byte[]? request = Frame.Read(pipe);
            if (request is null) break; // client disconnected

            using var reqStream = new MemoryStream(request, writable: false);
            using var reqReader = new BinaryReader(reqStream);
            ushort op = reqReader.ReadUInt16();

            using var respStream = new MemoryStream();
            using var respWriter = new BinaryWriter(respStream);
            respWriter.Write((byte)0); // optimistic OK status

            try
            {
                keepRunning = dispatch(op, reqReader, respWriter);
            }
            catch (Exception ex)
            {
                respStream.SetLength(0);
                respWriter.Write((byte)1); // error status
                respWriter.Write(ex.Message);
                keepRunning = true; // report the error; let the orchestrator decide to stop
            }

            respWriter.Flush();
            Frame.Write(pipe, respStream.GetBuffer().AsSpan(0, (int)respStream.Length));
        }
    }
}
