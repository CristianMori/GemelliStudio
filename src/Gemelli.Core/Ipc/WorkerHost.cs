using System.Diagnostics;

namespace Gemelli.Core.Ipc;

/// <summary>
/// Owns one worker process and its pipe connection. Captures the worker's stdout/stderr into a
/// capped ring buffer (the native runtimes are very chatty), watches for process death, and turns
/// connect failures and mid-call disconnects into a clear <see cref="TwinWorkerException"/> that
/// names the worker, its exit code, and the tail of its log — instead of a bare EndOfStreamException.
/// </summary>
internal sealed class WorkerHost : IDisposable
{
    private const int LogTailLines = 60;

    private readonly string _name;
    private readonly Process _process;
    private readonly object _logLock = new();
    private readonly Queue<string> _log = new();
    private PipeClient? _pipe;
    private bool _disposed;

    public string Name => _name;
    public bool IsAlive => !_process.HasExited;

    private WorkerHost(string name, Process process)
    {
        _name = name;
        _process = process;
    }

    /// <summary>Starts the worker process and connects to its pipe (clear error if it dies en route).</summary>
    public static WorkerHost Launch(
        string name, string exePath, string pipeName,
        IReadOnlyDictionary<string, string>? env = null, int connectTimeoutMs = 30_000)
    {
        var psi = new ProcessStartInfo
        {
            FileName = exePath,
            UseShellExecute = false,
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            CreateNoWindow = true, // don't pop a console window for each worker process
        };
        psi.ArgumentList.Add(pipeName);
        if (env is not null)
            foreach (var (k, v) in env) psi.Environment[k] = v;

        Process process = Process.Start(psi)
            ?? throw new TwinWorkerException($"Failed to start {name} worker ('{exePath}').");

        var host = new WorkerHost(name, process);
        process.OutputDataReceived += (_, e) => host.Capture(e.Data);
        process.ErrorDataReceived += (_, e) => host.Capture(e.Data);
        process.BeginOutputReadLine();
        process.BeginErrorReadLine();

        try
        {
            host._pipe = new PipeClient(pipeName, connectTimeoutMs);
        }
        catch (Exception ex)
        {
            throw host.Fail("failed to connect to its pipe", ex);
        }
        return host;
    }

    /// <summary>Forwards a request to the worker, translating a pipe disconnect or low-level failure into
    /// a diagnostics-rich <see cref="TwinWorkerException"/> (exit code + log tail) via <see cref="Fail"/>.</summary>
    public BinaryReader Request(ushort op, Action<BinaryWriter>? writeArgs = null)
    {
        if (_pipe is null) throw new TwinWorkerException($"{_name} worker pipe is not connected.");
        if (_process.HasExited) throw Fail($"has exited (code {_process.ExitCode}) before op {op}");

        try
        {
            return _pipe.Request(op, writeArgs);
        }
        catch (TwinWorkerDisconnectedException ex)
        {
            throw Fail($"crashed during op {op}", ex); // enrich with exit code + log tail
        }
        catch (TwinWorkerException) { throw; } // worker-reported error: already has a clear message
        catch (Exception ex)
        {
            throw Fail($"call (op {op}) failed", ex);
        }
    }

    /// <summary>Convenience for requests whose reply carries no payload.</summary>
    public void Send(ushort op, Action<BinaryWriter>? writeArgs = null) => Request(op, writeArgs).Dispose();

    /// <summary>Builds the enriched failure exception: waits briefly for the process to exit so the exit
    /// code and final log lines are available, then appends the captured log tail to the message.</summary>
    private TwinWorkerException Fail(string what, Exception? inner = null)
    {
        bool exited = _process.WaitForExit(500); // let it flush exit + final log lines
        string code = exited ? $", exit code {_process.ExitCode}" : " (still running)";
        string tail = LogTail();
        string msg = $"{_name} worker {what}{code}." +
                     (tail.Length > 0 ? $"\n--- {_name} log tail ---\n{tail}\n------------------------" : "");
        return inner is null ? new TwinWorkerException(msg) : new TwinWorkerException($"{msg}\n({inner.Message})");
    }

    /// <summary>Appends one stdout/stderr line to the capped ring buffer, dropping the oldest once full.</summary>
    private void Capture(string? line)
    {
        if (line is null) return;
        lock (_logLock)
        {
            _log.Enqueue(line);
            while (_log.Count > LogTailLines) _log.Dequeue();
        }
    }

    /// <summary>Recent worker output (last <see cref="LogTailLines"/> lines).</summary>
    public string LogTail()
    {
        lock (_logLock) return string.Join('\n', _log);
    }

    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;

        _pipe?.Dispose();
        try
        {
            if (!_process.WaitForExit(5_000))
                _process.Kill(entireProcessTree: true);
        }
        catch { /* best effort */ }
        finally { _process.Dispose(); }
    }
}
