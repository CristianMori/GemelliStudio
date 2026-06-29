using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Styling;
using Avalonia.Themes.Fluent;

namespace Gemelli.Studio;

/// <summary>Code-only Avalonia application (no XAML): installs the Fluent dark theme + main window.</summary>
public sealed class App : Application
{
    /// <summary>Registers the Fluent theme styles and forces the dark variant.</summary>
    public override void Initialize()
    {
        Styles.Add(new FluentTheme());
        RequestedThemeVariant = ThemeVariant.Dark;
    }

    /// <summary>Attaches the <see cref="MainWindow"/> once the classic desktop lifetime is ready.</summary>
    public override void OnFrameworkInitializationCompleted()
    {
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
            desktop.MainWindow = new MainWindow();
        base.OnFrameworkInitializationCompleted();
    }
}
