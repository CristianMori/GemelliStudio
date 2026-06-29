namespace Gemelli.Core;

/// <summary>
/// Resolves the worker executables. Prefers an explicit override, then a copy next to the
/// orchestrator (published layout), then the sibling project build output located by walking up to
/// the repository root (the directory containing <c>Gemelli.slnx</c>) and reusing the current
/// build's config/target-framework folders.
/// </summary>
public static class TwinHostLocator
{
    /// <summary>Locates the physics worker (ovphysx host) executable.</summary>
    public static string PhysicsHost(string? overridePath = null) =>
        Resolve(overridePath, projectFolder: "Gemelli.Physics", assembly: "Gemelli.PhysicsHost", distSubdir: "physics");

    /// <summary>Locates the render worker (ovrtx host) executable.</summary>
    public static string RenderHost(string? overridePath = null) =>
        Resolve(overridePath, projectFolder: "Gemelli.Render", assembly: "Gemelli.RenderHost", distSubdir: "render");

    /// <summary>Resolves a worker exe by trying, in order: the explicit override, the assembled-app layout
    /// beside the orchestrator, the sibling project's build output reusing the current bin tail, and finally
    /// a best-match search of the sibling project's bin tree. Throws if none exist.</summary>
    private static string Resolve(string? overridePath, string projectFolder, string assembly, string distSubdir)
    {
        if (!string.IsNullOrEmpty(overridePath))
            return overridePath;

        string exeName = assembly + (OperatingSystem.IsWindows() ? ".exe" : "");

        // 1. Assembled-app layout: each worker lives in its own subfolder beside the orchestrator
        //    (dist/<Config>/<distSubdir>/) so USD.NET's native libs in the Studio root don't shadow it.
        //    Also accept a flat copy next to the orchestrator (e.g. a hand-assembled single folder).
        string sub = Path.Combine(AppContext.BaseDirectory, distSubdir, exeName);
        if (File.Exists(sub)) return sub;
        string local = Path.Combine(AppContext.BaseDirectory, exeName);
        if (File.Exists(local)) return local;

        // 2. Sibling project output. Reuse the orchestrator's bin tail (e.g. bin/x64/Release/net10.0) so
        //    platform/config/tfm match. If the orchestrator was built with a RID (trailing "win-x64"
        //    subfolder) but the workers were not, also try the tail with that RID segment stripped.
        string baseDir = AppContext.BaseDirectory.TrimEnd(Path.DirectorySeparatorChar);
        string[] parts = baseDir.Split(Path.DirectorySeparatorChar);
        int binIndex = Array.LastIndexOf(parts, "bin");
        DirectoryInfo? root = FindRepoRoot(new DirectoryInfo(baseDir));
        if (binIndex > 0 && root is not null)
        {
            var tails = new List<string> { string.Join(Path.DirectorySeparatorChar, parts[binIndex..]) };
            if (parts[^1].Contains('-')) // looks like a RID (win-x64 / linux-x64) — workers may have none
                tails.Add(string.Join(Path.DirectorySeparatorChar, parts[binIndex..^1]));
            foreach (string tail in tails)
            {
                string candidate = Path.Combine(root.FullName, "src", projectFolder, tail, exeName);
                if (File.Exists(candidate)) return candidate;
            }

            // 3. Layouts differ (e.g. orchestrator built with a RID under bin/Release/net.../win-x64 while
            //    the worker uses an x64 platform folder bin/x64/Release/net...). Search the sibling project's
            //    whole bin tree for the exe, preferring a path that matches our config + target framework.
            string binRoot = Path.Combine(root.FullName, "src", projectFolder, "bin");
            if (Directory.Exists(binRoot))
            {
                string? config = parts.FirstOrDefault(p => p is "Debug" or "Release");
                string? tfm = parts.FirstOrDefault(p => p.StartsWith("net", StringComparison.OrdinalIgnoreCase));
                string[] found = Directory.GetFiles(binRoot, exeName, SearchOption.AllDirectories);
                string? best = found
                    .OrderByDescending(f => config is not null && f.Contains(config, StringComparison.OrdinalIgnoreCase))
                    .ThenByDescending(f => tfm is not null && f.Contains(tfm, StringComparison.OrdinalIgnoreCase))
                    .ThenByDescending(File.GetLastWriteTimeUtc)
                    .FirstOrDefault();
                if (best is not null) return best;
            }
        }

        throw new FileNotFoundException(
            $"Could not locate worker '{exeName}'. Build {projectFolder} (or the whole solution), " +
            "or pass an explicit host path in SimulationOptions.");
    }

    /// <summary>Walks up from <paramref name="start"/> to the directory containing <c>Gemelli.slnx</c>, or null.</summary>
    private static DirectoryInfo? FindRepoRoot(DirectoryInfo start)
    {
        for (DirectoryInfo? dir = start; dir is not null; dir = dir.Parent)
        {
            if (File.Exists(Path.Combine(dir.FullName, "Gemelli.slnx")))
                return dir;
        }
        return null;
    }
}
