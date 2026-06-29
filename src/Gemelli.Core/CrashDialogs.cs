using System.Runtime.InteropServices;
using System.Runtime.Versioning;

namespace Gemelli.Core;

/// <summary>
/// Suppresses the Windows "Application Error" dialog on a native fault, so worker processes that
/// crash (e.g. a native fail-fast) terminate with an exit code instead of blocking on a modal popup.
/// </summary>
public static class CrashDialogs
{
    [Flags]
    private enum ErrorModes : uint
    {
        SemFailCriticalErrors = 0x0001,
        SemNoGpFaultErrorBox = 0x0002,
        SemNoOpenFileErrorBox = 0x8000,
    }

    // Win32 SetErrorMode: controls which fault dialogs the process shows; returns the previous mode.
    [SupportedOSPlatform("windows")]
    [DllImport("kernel32.dll")]
    private static extern uint SetErrorMode(ErrorModes mode);

    /// <summary>Disables the critical-error, general-protection-fault, and open-file error dialogs for this
    /// process so a native crash fails fast with an exit code instead of hanging on a modal popup. No-op off Windows.</summary>
    public static void Suppress()
    {
        if (OperatingSystem.IsWindows())
            SetErrorMode(ErrorModes.SemFailCriticalErrors | ErrorModes.SemNoGpFaultErrorBox | ErrorModes.SemNoOpenFileErrorBox);
    }
}
