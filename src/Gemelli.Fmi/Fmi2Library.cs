using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace Gemelli.Fmi;

/// <summary>
/// Raw binding to one FMU's FMI 2.0 co-simulation C API: resolves the <c>fmi2*</c> exports from the
/// model's native library and exposes them as typed calls. One instance per loaded library; the
/// higher-level lifecycle (instantiate → setup → step → terminate) lives in <see cref="Fmu2Instance"/>.
/// </summary>
internal sealed unsafe class Fmi2Library : IDisposable
{
    // fmi2CallbackFunctions: five pointers the FMU may call back into. Real (non-null) allocate/free
    // callbacks matter: FMUs exported by industrial tools routinely allocate through them.
    [StructLayout(LayoutKind.Sequential)]
    private struct Callbacks
    {
        public IntPtr Logger;
        public IntPtr AllocateMemory;
        public IntPtr FreeMemory;
        public IntPtr StepFinished;
        public IntPtr ComponentEnvironment;
    }

    // The logger is variadic in C; we declare only the fixed prefix and ignore the varargs — safe
    // under cdecl (the caller owns stack cleanup). Message text is forwarded to stderr.
    [UnmanagedCallersOnly(CallConvs = [typeof(CallConvCdecl)])]
    private static void LoggerCallback(IntPtr env, IntPtr instanceName, int status, IntPtr category, IntPtr message)
    {
        try
        {
            string name = Marshal.PtrToStringUTF8(instanceName) ?? "?";
            string msg = Marshal.PtrToStringUTF8(message) ?? "";
            Console.Error.WriteLine($"[fmu {name}] status={status} {msg}");
        }
        catch { /* never throw back into native code */ }
    }

    [UnmanagedCallersOnly(CallConvs = [typeof(CallConvCdecl)])]
    private static void* AllocCallback(nuint nobj, nuint size) => NativeMemory.AllocZeroed(nobj, size);

    [UnmanagedCallersOnly(CallConvs = [typeof(CallConvCdecl)])]
    private static void FreeCallback(void* ptr) => NativeMemory.Free(ptr);

    private readonly IntPtr _lib;
    // In unmanaged memory: the FMI spec allows an FMU to hold on to the callbacks pointer for the
    // lifetime of the component, so it must never move or be freed while instances exist.
    private readonly Callbacks* _callbacks;

    private readonly delegate* unmanaged[Cdecl]<byte*, int, byte*, byte*, Callbacks*, int, int, IntPtr> _instantiate;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, void> _freeInstance;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, int, double, double, int, double, int> _setupExperiment;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, int> _enterInitializationMode;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, int> _exitInitializationMode;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, int> _terminate;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, uint*, nuint, double*, int> _setReal;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, uint*, nuint, double*, int> _getReal;
    private readonly delegate* unmanaged[Cdecl]<IntPtr, double, double, int, int> _doStep;

    /// <summary>Loads the FMU's native library and resolves the co-simulation entry points.</summary>
    public Fmi2Library(string libraryPath)
    {
        _lib = NativeLibrary.Load(libraryPath);
        _instantiate = (delegate* unmanaged[Cdecl]<byte*, int, byte*, byte*, Callbacks*, int, int, IntPtr>)Export("fmi2Instantiate");
        _freeInstance = (delegate* unmanaged[Cdecl]<IntPtr, void>)Export("fmi2FreeInstance");
        _setupExperiment = (delegate* unmanaged[Cdecl]<IntPtr, int, double, double, int, double, int>)Export("fmi2SetupExperiment");
        _enterInitializationMode = (delegate* unmanaged[Cdecl]<IntPtr, int>)Export("fmi2EnterInitializationMode");
        _exitInitializationMode = (delegate* unmanaged[Cdecl]<IntPtr, int>)Export("fmi2ExitInitializationMode");
        _terminate = (delegate* unmanaged[Cdecl]<IntPtr, int>)Export("fmi2Terminate");
        _setReal = (delegate* unmanaged[Cdecl]<IntPtr, uint*, nuint, double*, int>)Export("fmi2SetReal");
        _getReal = (delegate* unmanaged[Cdecl]<IntPtr, uint*, nuint, double*, int>)Export("fmi2GetReal");
        _doStep = (delegate* unmanaged[Cdecl]<IntPtr, double, double, int, int>)Export("fmi2DoStep");

        _callbacks = (Callbacks*)NativeMemory.AllocZeroed((nuint)sizeof(Callbacks));
        _callbacks->Logger = (IntPtr)(delegate* unmanaged[Cdecl]<IntPtr, IntPtr, int, IntPtr, IntPtr, void>)&LoggerCallback;
        _callbacks->AllocateMemory = (IntPtr)(delegate* unmanaged[Cdecl]<nuint, nuint, void*>)&AllocCallback;
        _callbacks->FreeMemory = (IntPtr)(delegate* unmanaged[Cdecl]<void*, void>)&FreeCallback;
    }

    private IntPtr Export(string name)
    {
        if (!NativeLibrary.TryGetExport(_lib, name, out IntPtr fn))
            throw new FmiException($"FMU library is missing required export '{name}' (co-simulation FMI 2.0).");
        return fn;
    }

    private const int Fmi2CoSimulation = 1;

    /// <summary>fmi2Instantiate for co-simulation. Returns the component handle (throws on null).</summary>
    public IntPtr Instantiate(string instanceName, string guid, string resourceUri)
    {
        byte[] name = Encoding.UTF8.GetBytes(instanceName + "\0");
        byte[] g = Encoding.UTF8.GetBytes(guid + "\0");
        byte[] uri = Encoding.UTF8.GetBytes(resourceUri + "\0");
        fixed (byte* pName = name, pGuid = g, pUri = uri)
        {
            IntPtr c = _instantiate(pName, Fmi2CoSimulation, pGuid, pUri, _callbacks, /*visible*/ 0, /*loggingOn*/ 0);
            if (c == IntPtr.Zero)
                throw new FmiException($"fmi2Instantiate returned NULL for '{instanceName}'.");
            return c;
        }
    }

    public void FreeInstance(IntPtr component) => _freeInstance(component);

    public void SetupExperiment(IntPtr c, double startTime) =>
        Check(_setupExperiment(c, 0, 0.0, startTime, 0, 0.0), "fmi2SetupExperiment");

    public void EnterInitializationMode(IntPtr c) => Check(_enterInitializationMode(c), "fmi2EnterInitializationMode");
    public void ExitInitializationMode(IntPtr c) => Check(_exitInitializationMode(c), "fmi2ExitInitializationMode");
    public void Terminate(IntPtr c) => Check(_terminate(c), "fmi2Terminate");

    public void SetReal(IntPtr c, ReadOnlySpan<uint> refs, ReadOnlySpan<double> values)
    {
        if (refs.IsEmpty) return;
        fixed (uint* pr = refs)
        fixed (double* pv = values)
            Check(_setReal(c, pr, (nuint)refs.Length, pv), "fmi2SetReal");
    }

    public void GetReal(IntPtr c, ReadOnlySpan<uint> refs, Span<double> values)
    {
        if (refs.IsEmpty) return;
        fixed (uint* pr = refs)
        fixed (double* pv = values)
            Check(_getReal(c, pr, (nuint)refs.Length, pv), "fmi2GetReal");
    }

    public void DoStep(IntPtr c, double currentTime, double stepSize) =>
        Check(_doStep(c, currentTime, stepSize, 1), "fmi2DoStep");

    // fmi2OK = 0, fmi2Warning = 1 (proceed); anything above is a failure.
    private static void Check(int status, string call)
    {
        if (status > 1) throw new FmiException($"{call} failed with fmi2Status {status}.");
    }

    public void Dispose()
    {
        NativeLibrary.Free(_lib);
        NativeMemory.Free(_callbacks);
    }
}

/// <summary>Raised for FMU load, instantiation, or co-simulation call failures.</summary>
public sealed class FmiException : Exception
{
    public FmiException(string message) : base(message) { }
    public FmiException(string message, Exception inner) : base(message, inner) { }
}
