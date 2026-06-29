using Microsoft.CodeAnalysis.CSharp.Scripting;
using Microsoft.CodeAnalysis.Scripting;
using Gemelli.Core.Control;
using Gemelli.Core.Sensors;

namespace Gemelli.Scripting;

/// <summary>
/// A controller that runs a user C# behavior script (<c>.csx</c>) once per frame. The script is
/// compiled once via Roslyn into a reusable delegate (recompiled when the file changes on disk, for
/// hot-reload). A compile error is reported once and the controller becomes a no-op until the file is
/// fixed — a bad script never crashes the twin.
/// </summary>
public sealed class ScriptController : IController, IDisposable
{
    private readonly string _path;
    private readonly bool _hotReload;
    private ScriptRunner<object>? _runner;
    private DateTime _loadedStamp;
    private string? _lastError;

    /// <summary>Binds to a script file (resolved to an absolute path); empty path means source-compiled.</summary>
    public ScriptController(string scriptPath, bool hotReload = true)
    {
        _path = string.IsNullOrEmpty(scriptPath) ? "" : Path.GetFullPath(scriptPath);
        _hotReload = hotReload;
    }

    /// <summary>Compiles a script source string directly (no file); used by tests and the server.</summary>
    public static ScriptController FromSource(string source)
    {
        var c = new ScriptController(scriptPath: "", hotReload: false);
        c._runner = Compile(source);
        return c;
    }

    /// <summary>Compiles the script up front so the first step has a ready delegate.</summary>
    public void OnStart(ISimApi sim) => EnsureCompiled();

    /// <summary>Runs the script once for this frame (recompiling first if hot-reload is on).</summary>
    public void OnPreStep(ISimApi sim)
    {
        if (_hotReload) EnsureCompiled();
        if (_runner is null) return;

        var globals = new ScriptGlobals { sim = sim, frame = sim.FrameCount, time = sim.SimTime };
        try
        {
            _runner(globals).GetAwaiter().GetResult();
        }
        catch (Exception ex)
        {
            // A runtime error in the script is reported once per distinct message, never fatal.
            ReportOnce($"[script runtime error] {ex.Message}");
        }
    }

    /// <summary>(Re)compiles the script when no delegate exists yet or the file's write time changed.</summary>
    private void EnsureCompiled()
    {
        if (string.IsNullOrEmpty(_path)) return; // source-compiled instance
        DateTime stamp;
        try { stamp = File.GetLastWriteTimeUtc(_path); }
        catch { return; }

        if (_runner is not null && stamp == _loadedStamp) return;

        try
        {
            string source = File.ReadAllText(_path);
            _runner = Compile(source);
            _loadedStamp = stamp;
            _lastError = null;
            Console.WriteLine($"[script] compiled {Path.GetFileName(_path)}");
        }
        catch (CompilationErrorException ex)
        {
            _runner = null;
            ReportOnce($"[script compile error] {string.Join("; ", ex.Diagnostics)}");
        }
    }

    /// <summary>Builds the reusable Roslyn delegate with the core assemblies and namespaces scripts expect in scope.</summary>
    private static ScriptRunner<object> Compile(string source)
    {
        ScriptOptions options = ScriptOptions.Default
            .AddReferences(
                typeof(ISimApi).Assembly,         // Gemelli.Core
                typeof(ScriptGlobals).Assembly)   // Gemelli.Scripting
            .AddImports(
                "System", "System.Linq", "System.Collections.Generic", "System.Numerics",
                "Gemelli.Core.Control", "Gemelli.Core.Ipc", "Gemelli.Core.Sensors");

        return CSharpScript.Create<object>(source, options, typeof(ScriptGlobals)).CreateDelegate();
    }

    /// <summary>Prints an error to stderr, suppressing repeats of the same message (avoids per-frame spam).</summary>
    private void ReportOnce(string message)
    {
        if (message == _lastError) return;
        _lastError = message;
        Console.Error.WriteLine(message);
    }

    public void OnStop(ISimApi sim) { } // no-op: the script holds no per-run state to tear down
    public void Dispose() { }           // no-op: Roslyn delegate needs no explicit disposal
}
