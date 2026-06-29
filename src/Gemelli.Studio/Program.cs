using Avalonia;
using Gemelli.Core;

namespace Gemelli.Studio;

internal static class Program
{
    /// <summary>Optional headless self-test config (parsed from --selftest), consumed by MainWindow.</summary>
    public static SelfTestConfig? SelfTest;

    /// <summary>Process entry point: parses the optional self-test args then runs the Avalonia desktop app.</summary>
    [STAThread]
    public static void Main(string[] args)
    {
        CrashDialogs.Suppress();
        // --selftest <usd> <product> <outPng> [frames]: auto-start the twin in the UI, save a viewport
        // frame, and exit. Lets the full UI integration be verified without an interactive display.
        int i = Array.IndexOf(args, "--selftest");
        if (i >= 0 && args.Length >= i + 4)
            SelfTest = new SelfTestConfig(args[i + 1], args[i + 2], args[i + 3],
                args.Length >= i + 5 && int.TryParse(args[i + 4], out int f) ? f : 20);

        BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);
    }

    /// <summary>Builds the Avalonia app (platform auto-detected, trace logging) hosting <see cref="App"/>.</summary>
    public static AppBuilder BuildAvaloniaApp() =>
        AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .LogToTrace();
}

/// <summary>Headless self-test parameters.</summary>
public sealed record SelfTestConfig(string Usd, string Product, string OutPng, int Frames);
