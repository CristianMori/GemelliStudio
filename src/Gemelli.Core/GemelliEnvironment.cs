namespace Gemelli.Core;

/// <summary>Process-level helpers and well-known names for locating the native libraries.</summary>
public static class GemelliEnvironment
{
    /// <summary>Environment variable ovphysx uses to locate its native library.</summary>
    public const string OvPhysxLibEnvVar = "OVPHYSX_LIB";

    /// <summary>Environment variable pointing at the directory holding <see cref="OvrtxDllName"/>.</summary>
    public const string OvrtxDirEnvVar = "GEMELLI_OVRTX_DIR";

    /// <summary>Native loader file name for the ovrtx renderer (Windows).</summary>
    public const string OvrtxDllName = "ovrtx-dynamic.dll";

    /// <summary>
    /// True when the ovphysx native library path is configured via <see cref="OvPhysxLibEnvVar"/>.
    /// Used to gate tier-2 (live) work.
    /// </summary>
    public static bool IsOvPhysxConfigured =>
        !string.IsNullOrEmpty(Environment.GetEnvironmentVariable(OvPhysxLibEnvVar));

    /// <summary>Walks up from <paramref name="start"/> (default: the app base dir) to the repo root (the
    /// directory containing <c>Gemelli.slnx</c>), or null if not found.</summary>
    public static string? FindRepoRoot(string? start = null)
    {
        for (var d = new DirectoryInfo(start ?? AppContext.BaseDirectory); d is not null; d = d.Parent)
            if (File.Exists(Path.Combine(d.FullName, "Gemelli.slnx")))
                return d.FullName;
        return null;
    }

    /// <summary>
    /// Resolves the ovphysx native library: honours <see cref="OvPhysxLibEnvVar"/> if it points at a real
    /// file, otherwise falls back to the conventional <c>native/ovphysx/ovphysx/lib/ovphysx.dll</c> under
    /// the repo root. Returns null if neither exists (callers should surface a clear error).
    /// </summary>
    public static string? ResolveOvPhysxLibrary()
    {
        string? env = Environment.GetEnvironmentVariable(OvPhysxLibEnvVar);
        if (!string.IsNullOrEmpty(env) && File.Exists(env)) return env;
        string? root = FindRepoRoot();
        if (root is null) return null;
        string p = Path.Combine(root, "native", "ovphysx", "ovphysx", "lib", "ovphysx.dll");
        return File.Exists(p) ? p : null;
    }

    /// <summary>
    /// Resolves the directory containing <see cref="OvrtxDllName"/>: honours <see cref="OvrtxDirEnvVar"/>
    /// if valid, otherwise the conventional <c>native/ovrtx/bin</c> under the repo root. Null if neither
    /// has the DLL.
    /// </summary>
    public static string? ResolveOvrtxDirectory()
    {
        string? env = Environment.GetEnvironmentVariable(OvrtxDirEnvVar);
        if (!string.IsNullOrEmpty(env) && File.Exists(Path.Combine(env, OvrtxDllName))) return env;
        string? root = FindRepoRoot();
        if (root is null) return null;
        string p = Path.Combine(root, "native", "ovrtx", "bin");
        return File.Exists(Path.Combine(p, OvrtxDllName)) ? p : null;
    }
}
