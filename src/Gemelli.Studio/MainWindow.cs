using System.Diagnostics;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Presenters;
using Avalonia.Controls.Primitives;
using Avalonia.Controls.Shapes;
using Avalonia.Styling;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Avalonia.Platform.Storage;
using Avalonia.Threading;
using Gemelli.Core;
using Gemelli.Core.Control;
using Gemelli.Core.Ipc;
using Gemelli.Core.Sensors;
using Gemelli.Scripting;
using Gemelli.Viewport;
using Path = System.IO.Path;

namespace Gemelli.Studio;

/// <summary>
/// Gemelli Studio — a dark-themed 3D-editor shell over the shared <see cref="TwinService"/>:
/// docked panels (outliner · viewport · inspector · timeline), an icon transport bar, a live viewport
/// with mouse camera orbit/pan/zoom, prim selection → live inspector, and a status/timeline strip.
/// Built entirely in code (no XAML).
/// </summary>
public sealed class MainWindow : Window
{
    // ---- Palette (refined dark, teal accent for "twin") ----
    private static readonly IBrush Bg = B("#14161B");
    private static readonly IBrush Panel = B("#1B1E25");
    private static readonly IBrush PanelAlt = B("#22262E");
    private static readonly IBrush Border = B("#2B313B");
    private static readonly IBrush Text = B("#D7DBE2");
    private static readonly IBrush TextDim = B("#828B99");
    private static readonly IBrush Accent = B("#2EC4B6");
    private static readonly IBrush AccentDim = B("#1E6E66");
    private static IBrush B(string hex) => new SolidColorBrush(Color.Parse(hex));

    private readonly TwinService _twin = new();
    private readonly OrbitCamera _camera = new();
    private const string CameraPath = "/OmniverseKit_Persp";

    private ComboBox _sceneCombo = null!;
    private readonly System.Collections.ObjectModel.ObservableCollection<SceneItem> _scenes = new();
    private TextBox _productBox = null!;

    /// <summary>A scene entry: display name in the dropdown, full path used to load.</summary>
    private sealed record SceneItem(string Name, string Path) { public override string ToString() => Name; }

    private string CurrentScenePath => (_sceneCombo.SelectedItem as SceneItem)?.Path ?? "";
    private ComboBox _deviceBox = null!;
    private ComboBox _resBox = null!;
    private ComboBox _viewportModeBox = null!;
    private RasterViewport? _raster;            // non-null while the fast rasterized viewport is active

    // Pacing settings (edited in the Settings popup; applied live while running and seeded into options on Start).
    private double _timeScale = 1.0;            // sim-time : wall-clock factor (10 = 10× accelerated)
    private double _timeStepMs = 1000.0 / 60.0; // fixed physics timestep, milliseconds
    private Window? _settingsWindow;            // the non-modal Settings pane, while open
    private Button _startBtn = null!, _playBtn = null!, _pauseBtn = null!, _stepBtn = null!, _stopBtn = null!;
    private Image _viewport = null!;
    private TextBlock _viewportOverlay = null!;
    private TextBlock _viewportHint = null!;
    private ListBox _outliner = null!;
    private StackPanel _inspector = null!;
    private TextBlock _statusLeft = null!, _statusRight = null!;

    // ---- inspector live transform editing + USD save-back ----
    private Button _saveBtn = null!;
    private string? _inspectedPath;                 // path the editable inspector was built for
    private TextBox? _posX, _posY, _posZ;
    private TextBlock? _liveReadout;
    private bool _editingPose;                       // suppress live overwrite while the user types

    private WriteableBitmap? _viewportBitmap;
    private long _lastFrameTick;
    private double _fps;
    private volatile string? _activeProduct;
    private volatile RenderVarData? _pendingColor;
    private int _postScheduled;

    // ---- sensor preview (fixed sensor camera: depth / segmentation / color) ----
    private Image _sensorImage = null!;
    private TextBlock _sensorHint = null!;
    private TextBlock _sensorOverlay = null!;
    private ComboBox _sensorModeCombo = null!;
    private Button _recordBtn = null!;
    private WriteableBitmap? _sensorBitmap;
    private volatile string? _sensorProduct;       // null = scene has no depth/sensor product
    private volatile CapturedFrame? _pendingSensorFrame;
    private int _sensorPostScheduled;
    private SensorRecorder? _recorder;
    private int _recordIndex;

    private Point _lastPointer;
    private bool _dragging;

    // ---- viewport object pick + drag ----
    private Grid _viewportHost = null!;
    private bool _objectDrag;
    private string? _dragPath;
    private float _dragDist;
    private System.Numerics.Vector3 _dragTarget;
    private float[] _dragQuat = [0, 0, 0, 1];
    private float _dragPlaneZ;                     // height plane the free-drag slides on
    private System.Numerics.Vector3 _dragGroundOffset; // body-centre minus the ground point first grabbed
    private float _camFocal = 18.147562f;       // render camera projection (read from scene at Start)
    private float _camHorizAperture = 20.955f;

    // ---- articulation (robot) IK-drag ----
    private string? _robotPath;                  // articulation root, e.g. /World/robot (null = no robot)
    private int _robotLinkCount;
    private IkDragController? _ikController;      // non-null while IK-dragging a robot link

    // ---- 3-axis transform gizmo (screen-space overlay on the selected rigid body) ----
    private Canvas _gizmoCanvas = null!;
    private int _gizmoAxis = -1;                  // 0=X 1=Y 2=Z while axis-dragging, else -1
    private System.Numerics.Vector3 _gizmoPivot;  // dragged body's world position (accumulated)
    private float[] _gizmoQuat = [0, 0, 0, 1];
    private static readonly System.Numerics.Vector3[] AxisDirs =
    {
        new(1, 0, 0), new(0, 1, 0), new(0, 0, 1),
    };
    private static readonly IBrush[] AxisBrushes = { B("#E5534B"), B("#3FB950"), B("#4C8DFF") }; // X r, Y g, Z b

    private Border _scriptPanel = null!;
    private TextBox _scriptBox = null!;
    private TextBlock _scriptStatus = null!;
    private Button _runScriptBtn = null!, _stopScriptBtn = null!;

    private const string DemoScript = """
        // Runs once per frame. In scope: sim, frame, time, ReadController, ReadKeyboard, MoveTcp, LinkPosition.
        // Drive the robot's TCP (end-effector) with the keyboard (WASD + Q/E) or an Xbox controller.
        // Each key is a bool: kb.W, kb.A, kb.S, kb.D, kb.Q, kb.E (also kb.Space, kb.Left, kb["1"], ...).
        var kb  = ReadKeyboard();
        var mov = ReadController();

        float fwd    = (kb.W ? 1f : 0f) - (kb.S ? 1f : 0f) + mov.LeftY;                       // forward / back
        float strafe = (kb.D ? 1f : 0f) - (kb.A ? 1f : 0f) + mov.LeftX;                       // right / left
        float lift   = (kb.E ? 1f : 0f) - (kb.Q ? 1f : 0f) + (mov.RightTrigger - mov.LeftTrigger); // up / down

        var tcp = LinkPosition("/World/robot", 8);
        if (tcp.HasValue)
        {
            float speed = 0.02f;
            MoveTcp("/World/robot", 8,
                tcp.Value.X + Math.Clamp(fwd, -1f, 1f) * speed,
                tcp.Value.Y + Math.Clamp(strafe, -1f, 1f) * speed,
                tcp.Value.Z + Math.Clamp(lift, -1f, 1f) * speed);
        }
        """;

    /// <summary>Builds the window chrome, wires twin events, the keyboard pump, and the 200ms status tick.</summary>
    public MainWindow()
    {
        Title = "Gemelli Studio";
        Width = 1320;
        Height = 820;
        Background = Bg;
        Content = BuildLayout();

        _twin.FrameProduced += OnFrameProduced;
        _twin.Faulted += OnFaulted;
        Closed += (_, _) => { StopRecording(); StopRasterViewport(); _twin.FrameProduced -= OnFrameProduced; _twin.Faulted -= OnFaulted; _twin.Dispose(); };

        // Feed keyboard state into the shared Keyboard store so scripts (sim thread) can read it.
        // handledEventsToo so keys register even if a focused control also handled them.
        AddHandler(KeyDownEvent, (_, e) => Keyboard.SetKey(e.Key.ToString(), true), RoutingStrategies.Tunnel | RoutingStrategies.Bubble, handledEventsToo: true);
        AddHandler(KeyUpEvent, (_, e) => Keyboard.SetKey(e.Key.ToString(), false), RoutingStrategies.Tunnel | RoutingStrategies.Bubble, handledEventsToo: true);
        Deactivated += (_, _) => Keyboard.Clear(); // release all when the window loses focus

        var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(200) };
        timer.Tick += (_, _) => RefreshStatusAndInspector();
        timer.Start();

        if (Program.SelfTest is { } st) Opened += (_, _) => RunSelfTest(st);
    }

    // ====================================================================== layout

    /// <summary>Assembles the root grid: header row, 3-pane body (outliner · viewport · right column), collapsible script panel, status bar.</summary>
    private Control BuildLayout()
    {
        var root = new Grid
        {
            RowDefinitions = new RowDefinitions("Auto,*,Auto,Auto"),
        };

        Control header = Header();
        Grid.SetRow(header, 0);
        root.Children.Add(header);

        var body = new Grid { ColumnDefinitions = new ColumnDefinitions("220,4,*,4,300") };
        Grid.SetRow(body, 1);
        body.Children.Add(Place(OutlinerPanel(), 0));
        body.Children.Add(Splitter(1));
        body.Children.Add(Place(ViewportPanel(), 2));
        body.Children.Add(Splitter(3));
        body.Children.Add(Place(RightColumn(), 4));
        root.Children.Add(body);

        _scriptPanel = ScriptPanel();
        _scriptPanel.IsVisible = false; // toggled by the header "Script" button
        Grid.SetRow(_scriptPanel, 2);
        root.Children.Add(_scriptPanel);

        Control status = StatusBar();
        Grid.SetRow(status, 3);
        root.Children.Add(status);
        return root;
    }

    /// <summary>Assigns a grid column to a child and returns it (fluent helper for inline adds).</summary>
    private static Control Place(Control c, int col) { Grid.SetColumn(c, col); return c; }

    /// <summary>A draggable column splitter at the given grid column.</summary>
    private static GridSplitter Splitter(int col)
    {
        var s = new GridSplitter { Background = Border, ResizeDirection = GridResizeDirection.Columns };
        Grid.SetColumn(s, col);
        return s;
    }

    /// <summary>Top bar: brand, centred transport buttons, and the right-hand scene/config row (scene picker, device, viewport mode, render scale, settings/script/save/start).</summary>
    private Control Header()
    {
        var bar = new Grid { ColumnDefinitions = new ColumnDefinitions("Auto,*,Auto"), Height = 48, Background = Panel };

        var brand = new StackPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(14, 0), VerticalAlignment = VerticalAlignment.Center };
        brand.Children.Add(new Ellipse { Width = 12, Height = 12, Fill = Accent, VerticalAlignment = VerticalAlignment.Center });
        brand.Children.Add(new TextBlock { Text = "Gemelli", FontSize = 18, FontWeight = FontWeight.SemiBold, Foreground = Text, Margin = new Thickness(8, 0, 4, 0), VerticalAlignment = VerticalAlignment.Center });
        brand.Children.Add(new TextBlock { Text = "Studio", FontSize = 18, Foreground = TextDim, VerticalAlignment = VerticalAlignment.Center });
        bar.Children.Add(brand);

        // transport buttons — text labels with Fluent's default dark styling (clearly visible).
        var transport = new StackPanel { Orientation = Orientation.Horizontal, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center, Spacing = 6 };
        _playBtn = TransportButton("Play", () => _twin.Play());
        _pauseBtn = TransportButton("Pause", () => _twin.Pause());
        _stepBtn = TransportButton("Step", () => Task.Run(() => _twin.Step(1)));
        _stopBtn = TransportButton("Stop", OnStop);
        transport.Children.Add(_playBtn);
        transport.Children.Add(_pauseBtn);
        transport.Children.Add(_stepBtn);
        transport.Children.Add(_stopBtn);
        Grid.SetColumn(transport, 1);
        bar.Children.Add(transport);

        // scene config (right): a dropdown of scenes/ + a Browse button for any file on disk.
        string? sceneDir = SceneFolder();
        if (sceneDir is not null && Directory.Exists(sceneDir))
            foreach (string f in Directory.GetFiles(sceneDir, "*.usd*").OrderBy(f => f))
                _scenes.Add(new SceneItem(Path.GetFileName(f), f));

        _sceneCombo = new ComboBox
        {
            Width = 240, FontSize = 12, ItemsSource = _scenes, VerticalAlignment = VerticalAlignment.Center,
            PlaceholderText = "select a scene",
        };
        _sceneCombo.SelectedItem = _scenes.FirstOrDefault(s => s.Name.Contains("franka", StringComparison.OrdinalIgnoreCase))
                                   ?? _scenes.FirstOrDefault();
        var browseBtn = TransportButton("Browse", () => _ = BrowseScene());
        _productBox = Field("/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0", 200, "render product");
        _deviceBox = new ComboBox { ItemsSource = new[] { "cpu", "gpu", "auto" }, SelectedIndex = 0, Width = 76, VerticalAlignment = VerticalAlignment.Center, Foreground = Text };
        // Render scale: fewer pixels = less path-tracing = higher fps (GPU-bound). Native by default.
        _resBox = new ComboBox { ItemsSource = new[] { "Full res", "75%", "50%" }, SelectedIndex = 0, Width = 90, VerticalAlignment = VerticalAlignment.Center, Foreground = Text };
        // Viewport renderer: Fast = rasterized (cheap, ~60fps); RTX = ovrtx path-traced viewport.
        _viewportModeBox = new ComboBox { ItemsSource = new[] { "Fast", "RTX" }, SelectedIndex = 0, Width = 70, VerticalAlignment = VerticalAlignment.Center, Foreground = Text };
        _startBtn = AccentButton("Start", OnStart);

        var scriptToggle = TransportButton("Script", () => { _scriptPanel.IsVisible = !_scriptPanel.IsVisible; });
        var settingsBtn = TransportButton("⚙ Settings", OpenSettings);
        _saveBtn = TransportButton("Save USD", () => _ = SaveUsdSnapshot());

        var cfg = new StackPanel { Orientation = Orientation.Horizontal, Margin = new Thickness(0, 0, 12, 0), VerticalAlignment = VerticalAlignment.Center, Spacing = 6 };
        cfg.Children.Add(Label("Scene"));
        cfg.Children.Add(_sceneCombo);
        cfg.Children.Add(browseBtn);
        cfg.Children.Add(_deviceBox);
        cfg.Children.Add(_viewportModeBox);
        cfg.Children.Add(_resBox);
        cfg.Children.Add(settingsBtn);
        cfg.Children.Add(scriptToggle);
        cfg.Children.Add(_saveBtn);
        cfg.Children.Add(_startBtn);
        Grid.SetColumn(cfg, 2);
        bar.Children.Add(cfg);

        SetRunningControls(false);
        return bar;
    }

    /// <summary>Left panel listing the scene's rigid-body prim paths; selection drives the inspector.</summary>
    private Control OutlinerPanel()
    {
        _outliner = new ListBox { Background = Brushes.Transparent, BorderThickness = new Thickness(0) };
        _outliner.SelectionChanged += (_, _) => RefreshStatusAndInspector();

        // Explicit dark-theme item styling (default selection highlight is invisible against our palette).
        var item = new Style(s => s.OfType<ListBoxItem>());
        item.Setters.Add(new Setter(TemplatedControl.ForegroundProperty, Text));
        item.Setters.Add(new Setter(TemplatedControl.PaddingProperty, new Thickness(8, 5)));
        item.Setters.Add(new Setter(TemplatedControl.FontSizeProperty, 12.0));
        _outliner.Styles.Add(item);

        var selected = new Style(s => s.OfType<ListBoxItem>().Class(":selected").Template().OfType<ContentPresenter>());
        selected.Setters.Add(new Setter(ContentPresenter.BackgroundProperty, AccentDim));
        _outliner.Styles.Add(selected);

        var hover = new Style(s => s.OfType<ListBoxItem>().Class(":pointerover").Template().OfType<ContentPresenter>());
        hover.Setters.Add(new Setter(ContentPresenter.BackgroundProperty, PanelAlt));
        _outliner.Styles.Add(hover);

        return PanelBox("OUTLINER", new ScrollViewer { Content = _outliner });
    }

    /// <summary>The main 3D viewport: rendered image + gizmo canvas + hint/overlay, with mouse camera control wired on the host grid.</summary>
    private Control ViewportPanel()
    {
        _viewport = new Image { Stretch = Stretch.Uniform };
        _viewportOverlay = new TextBlock { Foreground = Accent, FontSize = 12, FontFamily = new FontFamily("Consolas, monospace"), Margin = new Thickness(10, 8), HorizontalAlignment = HorizontalAlignment.Left, VerticalAlignment = VerticalAlignment.Top };
        _viewportHint = new TextBlock { Text = "Set a scene and press  ▶ Start", Foreground = TextDim, FontSize = 14, HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center };

        var host = new Grid { Background = B("#0B0D10") };
        _viewportHost = host;
        _gizmoCanvas = new Canvas { IsHitTestVisible = false };
        host.Children.Add(_viewport);
        host.Children.Add(_gizmoCanvas);
        host.Children.Add(_viewportHint);
        host.Children.Add(_viewportOverlay);

        // Camera-only control: drag = orbit, Shift-drag = pan, wheel = zoom. (Object click-select/drag is
        // disabled for now — screen-space picking proved unreliable; edit transforms via the inspector.)
        host.PointerPressed += (_, e) => { _dragging = true; _lastPointer = e.GetPosition(host); };
        host.PointerReleased += (_, _) => _dragging = false;
        host.PointerMoved += (_, e) => { if (_dragging) OnViewportDrag(e.GetPosition(host), e.KeyModifiers); };
        host.PointerWheelChanged += (_, e) => { _camera.Zoom(e.Delta.Y > 0 ? 0.9f : 1.1f); PushCamera(); };

        var border = new Border { Background = B("#0B0D10"), BorderBrush = Border, BorderThickness = new Thickness(1), Child = host };
        return border;
    }

    // Right column: a fixed-height SENSOR preview on top, the INSPECTOR below.
    private Control RightColumn()
    {
        var col = new Grid { RowDefinitions = new RowDefinitions("320,*") };
        Control sensor = SensorPanel();
        Grid.SetRow(sensor, 0);
        col.Children.Add(sensor);
        Control inspector = InspectorPanel();
        Grid.SetRow(inspector, 1);
        col.Children.Add(inspector);
        return col;
    }

    /// <summary>Right-column top panel previewing the fixed sensor camera (color/depth/segmentation) with a record toggle.</summary>
    private Control SensorPanel()
    {
        _sensorImage = new Image { Stretch = Stretch.Uniform };
        _sensorHint = new TextBlock
        {
            Text = "Start a scene with a sensor camera\n(e.g. franka_studio) to preview depth.",
            Foreground = TextDim, FontSize = 11, TextAlignment = TextAlignment.Center, TextWrapping = TextWrapping.Wrap,
            HorizontalAlignment = HorizontalAlignment.Center, VerticalAlignment = VerticalAlignment.Center,
        };
        _sensorOverlay = new TextBlock { Foreground = Accent, FontSize = 10, FontFamily = new FontFamily("Consolas, monospace"), Margin = new Thickness(6, 4), HorizontalAlignment = HorizontalAlignment.Left, VerticalAlignment = VerticalAlignment.Top };

        var imageHost = new Grid { Background = B("#0B0D10"), MinHeight = 200 };
        imageHost.Children.Add(_sensorImage);
        imageHost.Children.Add(_sensorHint);
        imageHost.Children.Add(_sensorOverlay);

        _sensorModeCombo = new ComboBox { ItemsSource = new[] { "Color", "Depth", "Segmentation" }, SelectedIndex = 1, Width = 130, FontSize = 12, VerticalAlignment = VerticalAlignment.Center };
        _sensorModeCombo.SelectionChanged += (_, _) => RedrawSensor();
        _recordBtn = TransportButton("● Rec", ToggleRecording);

        var controls = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 6, Margin = new Thickness(8, 6) };
        controls.Children.Add(_sensorModeCombo);
        controls.Children.Add(_recordBtn);

        var dock = new DockPanel();
        var header = new TextBlock { Text = "SENSOR", Foreground = TextDim, FontSize = 11, FontWeight = FontWeight.SemiBold, Margin = new Thickness(12, 10, 12, 6) };
        DockPanel.SetDock(header, Dock.Top);
        DockPanel.SetDock(controls, Dock.Bottom);
        dock.Children.Add(header);
        dock.Children.Add(controls);
        var border = new Border { Background = B("#0B0D10"), BorderBrush = Border, BorderThickness = new Thickness(1), Child = imageHost, Margin = new Thickness(6, 0, 6, 4) };
        dock.Children.Add(border);

        return new Border { Background = Panel, Child = dock };
    }

    /// <summary>Right-column bottom panel hosting the per-prim editable transform inspector.</summary>
    private Control InspectorPanel()
    {
        _inspector = new StackPanel { Spacing = 6, Margin = new Thickness(2) };
        _inspector.Children.Add(new TextBlock { Text = "Select a prim in the outliner.", Foreground = TextDim, FontSize = 12, TextWrapping = TextWrapping.Wrap });
        return PanelBox("INSPECTOR", new ScrollViewer { Content = _inspector });
    }

    /// <summary>Collapsible bottom editor for the per-frame C# control script, with Run/Stop and a status line.</summary>
    private Border ScriptPanel()
    {
        _scriptBox = new TextBox
        {
            Text = DemoScript, AcceptsReturn = true, AcceptsTab = true,
            FontFamily = new FontFamily("Consolas, monospace"), FontSize = 12,
            Height = 150, TextWrapping = TextWrapping.NoWrap,
        };
        ScrollViewer.SetHorizontalScrollBarVisibility(_scriptBox, ScrollBarVisibility.Auto);
        ScrollViewer.SetVerticalScrollBarVisibility(_scriptBox, ScrollBarVisibility.Auto);
        _runScriptBtn = AccentButton("Run", RunScript);
        _stopScriptBtn = new Button { Content = "Stop", Padding = new Thickness(14, 6), VerticalAlignment = VerticalAlignment.Center, IsEnabled = false };
        _stopScriptBtn.Click += (_, _) => StopScript();
        _scriptStatus = new TextBlock { Foreground = TextDim, FontSize = 12, VerticalAlignment = VerticalAlignment.Center, Margin = new Thickness(10, 0) };

        var headerRow = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 8, Margin = new Thickness(12, 8, 12, 4) };
        headerRow.Children.Add(new TextBlock { Text = "SCRIPT  ·  C# per-frame (sim, frame, time in scope)", Foreground = TextDim, FontSize = 11, FontWeight = FontWeight.SemiBold, VerticalAlignment = VerticalAlignment.Center });
        headerRow.Children.Add(_runScriptBtn);
        headerRow.Children.Add(_stopScriptBtn);
        headerRow.Children.Add(_scriptStatus);

        var dock = new DockPanel();
        DockPanel.SetDock(headerRow, Dock.Top);
        dock.Children.Add(headerRow);
        _scriptBox.Margin = new Thickness(12, 0, 12, 10);
        dock.Children.Add(_scriptBox);

        return new Border { Background = Panel, BorderBrush = Border, BorderThickness = new Thickness(0, 1, 0, 0), Child = dock };
    }

    /// <summary>Compiles the editor text into a live per-frame controller and installs it on the running twin.</summary>
    private void RunScript()
    {
        if (!_twin.IsRunning) { _scriptStatus.Text = "Start the twin first."; return; }
        try
        {
            var controller = ScriptController.FromSource(_scriptBox.Text ?? "");
            _twin.SetLiveController(controller);
            _scriptStatus.Foreground = Accent;
            _scriptStatus.Text = "● running";
            _runScriptBtn.IsEnabled = false;
            _stopScriptBtn.IsEnabled = true;
        }
        catch (Exception ex)
        {
            _scriptStatus.Foreground = B("#E5736B");
            _scriptStatus.Text = "compile error: " + ex.Message.Split('\n')[0];
        }
    }

    /// <summary>Detaches any live script controller from the twin.</summary>
    private void StopScript()
    {
        _twin.SetLiveController(null);
        _scriptStatus.Foreground = TextDim;
        _scriptStatus.Text = "stopped";
        _runScriptBtn.IsEnabled = true;
        _stopScriptBtn.IsEnabled = false;
    }

    /// <summary>Bottom strip: left status text and a right-aligned monospace telemetry readout.</summary>
    private Control StatusBar()
    {
        var grid = new Grid { ColumnDefinitions = new ColumnDefinitions("*,Auto"), Height = 30, Background = Panel };
        _statusLeft = new TextBlock { Foreground = TextDim, FontSize = 12, Margin = new Thickness(12, 0), VerticalAlignment = VerticalAlignment.Center, Text = "Idle" };
        _statusRight = new TextBlock { Foreground = TextDim, FontSize = 12, FontFamily = new FontFamily("Consolas, monospace"), Margin = new Thickness(12, 0), VerticalAlignment = VerticalAlignment.Center };
        Grid.SetColumn(_statusRight, 1);
        grid.Children.Add(_statusLeft);
        grid.Children.Add(_statusRight);
        return grid;
    }

    /// <summary>Wraps content in a titled dark panel (header label docked above the body).</summary>
    private Control PanelBox(string title, Control content)
    {
        var header = new TextBlock { Text = title, Foreground = TextDim, FontSize = 11, FontWeight = FontWeight.SemiBold, Margin = new Thickness(12, 10, 12, 6) };
        var stack = new DockPanel();
        DockPanel.SetDock(header, Dock.Top);
        stack.Children.Add(header);
        content.Margin = new Thickness(6, 0, 6, 6);
        stack.Children.Add(content);
        return new Border { Background = Panel, Child = stack };
    }

    // ====================================================================== controls
    // Use Fluent's default Dark control styling for interactive controls (clearly visible); only the
    // Start button gets an accent fill. Custom dark backgrounds on Fluent controls render invisibly.

    /// <summary>A standard Fluent dark button wired to <paramref name="onClick"/>.</summary>
    private Button TransportButton(string text, Action onClick)
    {
        var b = new Button
        {
            Content = text, MinWidth = 60, Padding = new Thickness(12, 6),
            HorizontalContentAlignment = HorizontalAlignment.Center,
            VerticalAlignment = VerticalAlignment.Center,
        };
        b.Click += (_, _) => onClick();
        return b;
    }

    /// <summary>A filled accent (teal) button for primary actions.</summary>
    private Button AccentButton(string text, Action onClick)
    {
        var b = new Button
        {
            Content = text, Foreground = Brushes.Black, Background = Accent, FontWeight = FontWeight.SemiBold,
            Padding = new Thickness(18, 6), CornerRadius = new CornerRadius(4),
            VerticalAlignment = VerticalAlignment.Center,
        };
        b.Click += (_, _) => onClick();
        return b;
    }

    /// <summary>A watermarked text box of fixed width.</summary>
    private TextBox Field(string text, double width, string watermark) => new()
    {
        Text = text, Width = width, Watermark = watermark, VerticalAlignment = VerticalAlignment.Center, FontSize = 12,
    };

    /// <summary>A dimmed inline caption.</summary>
    private TextBlock Label(string t) => new() { Text = t, Foreground = TextDim, FontSize = 12, VerticalAlignment = VerticalAlignment.Center };

    // ====================================================================== settings popup
    // A non-modal pane for pacing: time-scale (sim-speed multiplier) and physics step. Both apply LIVE to a
    // running twin (TwinService reads them each loop iteration) and seed SimulationOptions on the next Start.

    private void OpenSettings()
    {
        if (_settingsWindow is not null) { _settingsWindow.Activate(); return; }

        var win = new Window
        {
            Title = "Simulation Settings", Width = 460, Height = 360, Background = Bg,
            CanResize = false, ShowInTaskbar = false,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
        };

        // ---- Time scale ----
        var scaleVal = new TextBlock { Foreground = Accent, FontSize = 14, FontWeight = FontWeight.SemiBold, MinWidth = 64, TextAlignment = TextAlignment.Right, VerticalAlignment = VerticalAlignment.Center };
        var scaleSlider = new Slider { Minimum = 0.1, Maximum = 10, Value = _timeScale, Width = 280, VerticalAlignment = VerticalAlignment.Center };
        void ApplyScale(double v)
        {
            _timeScale = Math.Round(v, 2);
            scaleVal.Text = $"{_timeScale:0.0}×";
            if (_twin.IsRunning) _twin.TimeScale = (float)_timeScale;
        }
        scaleSlider.ValueChanged += (_, e) => ApplyScale(e.NewValue);

        // ---- Physics step ----
        var stepVal = new TextBlock { Foreground = Accent, FontSize = 14, FontWeight = FontWeight.SemiBold, MinWidth = 64, TextAlignment = TextAlignment.Right, VerticalAlignment = VerticalAlignment.Center };
        var stepSlider = new Slider { Minimum = 1, Maximum = 1000.0 / 30.0, Value = _timeStepMs, Width = 280, VerticalAlignment = VerticalAlignment.Center };
        void ApplyStep(double ms)
        {
            _timeStepMs = Math.Round(ms, 2);
            stepVal.Text = $"{_timeStepMs:0.0} ms · {1000.0 / _timeStepMs:0} Hz";
            if (_twin.IsRunning) _twin.TimeStep = (float)(_timeStepMs / 1000.0);
        }
        stepSlider.ValueChanged += (_, e) => ApplyStep(e.NewValue);

        ApplyScale(_timeScale);
        ApplyStep(_timeStepMs);

        // ---- Live readout: measured physics ms → real-time ceiling ----
        var readout = new TextBlock { Foreground = TextDim, FontSize = 12, TextWrapping = TextWrapping.Wrap };
        var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(500) };
        timer.Tick += (_, _) => UpdateSettingsReadout(readout);
        timer.Start();
        UpdateSettingsReadout(readout);

        var panel = new StackPanel { Margin = new Thickness(20), Spacing = 12 };
        panel.Children.Add(SettingsRow("Time scale", "Sim-time speed-up. 10× = ten seconds of simulation per real second — capped by physics throughput.", scaleSlider, scaleVal));
        panel.Children.Add(SettingsRow("Physics step", "Fixed timestep. Smaller = more accurate/stable; larger = cheaper but can destabilize articulations.", stepSlider, stepVal));
        panel.Children.Add(new Border { Background = Panel, CornerRadius = new CornerRadius(4), Padding = new Thickness(12, 8), Child = readout });

        var resetBtn = TransportButton("Reset · 1× / 60 Hz", () => { scaleSlider.Value = 1.0; stepSlider.Value = 1000.0 / 60.0; });
        var closeBtn = AccentButton("Close", () => win.Close());
        var buttons = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 8, HorizontalAlignment = HorizontalAlignment.Right };
        buttons.Children.Add(resetBtn);
        buttons.Children.Add(closeBtn);
        panel.Children.Add(buttons);

        win.Content = panel;
        win.Closed += (_, _) => { timer.Stop(); _settingsWindow = null; };
        _settingsWindow = win;
        win.Show(this);
    }

    /// <summary>One settings entry: title + hint above a slider/value row.</summary>
    private Control SettingsRow(string title, string hint, Slider slider, TextBlock value)
    {
        var head = new StackPanel { Spacing = 2 };
        head.Children.Add(new TextBlock { Text = title, Foreground = Text, FontSize = 13, FontWeight = FontWeight.SemiBold });
        head.Children.Add(new TextBlock { Text = hint, Foreground = TextDim, FontSize = 11, TextWrapping = TextWrapping.Wrap });

        var row = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 12, VerticalAlignment = VerticalAlignment.Center };
        row.Children.Add(slider);
        row.Children.Add(value);

        var col = new StackPanel { Spacing = 4 };
        col.Children.Add(head);
        col.Children.Add(row);
        return col;
    }

    /// <summary>Refreshes the live readout comparing requested time-scale against the measured physics ceiling.</summary>
    private void UpdateSettingsReadout(TextBlock readout)
    {
        double physMs = _twin.LastPhysicsMs;
        if (!_twin.IsRunning || physMs <= 0)
        {
            readout.Text = "Start the twin to see the measured physics step time and the real-time ceiling.";
            return;
        }
        double ceiling = _timeStepMs / physMs;   // sim-time advanced per wall-second ≈ dt / step-cost
        string verdict = _timeScale > ceiling + 0.1
            ? $"Requested {_timeScale:0.0}× exceeds it — running flat-out near {ceiling:0.0}×."
            : $"Requested {_timeScale:0.0}× is within budget.";
        readout.Text = $"Measured physics: {physMs:0.0} ms/step  →  ceiling ≈ {ceiling:0.0}× real-time.\n{verdict}";
    }

    // Returns the scene to load, scaling every render product's resolution by the chosen factor (for fps)
    // into a sibling temp .usda. Native (Full res) or any non-text/binary scene returns the path unchanged.
    private string EffectiveScenePath()
    {
        double scale = (_resBox.SelectedItem as string) switch { "75%" => 0.75, "50%" => 0.5, _ => 1.0 };
        string scene = CurrentScenePath;
        if (scale >= 1.0 || string.IsNullOrEmpty(scene) || !scene.EndsWith(".usda", StringComparison.OrdinalIgnoreCase)) return scene;
        try
        {
            string text = File.ReadAllText(scene);
            string scaled = System.Text.RegularExpressions.Regex.Replace(text, @"int2 resolution = \((\d+),\s*(\d+)\)", m =>
            {
                int w = (int)(int.Parse(m.Groups[1].Value) * scale) & ~1; // even dims
                int h = (int)(int.Parse(m.Groups[2].Value) * scale) & ~1;
                return $"int2 resolution = ({w}, {h})";
            });
            // Write next to the original so any relative asset references still resolve.
            string tmp = Path.Combine(Path.GetDirectoryName(scene)!, $"_gemelli_res{(int)(scale * 100)}_{Path.GetFileName(scene)}");
            File.WriteAllText(tmp, scaled);
            return tmp;
        }
        catch { return scene; }
    }

    // Scans a (text) USD scene for a second render product that carries depth — i.e. a fixed sensor
    // camera distinct from the orbit viewport product. Returns its Hydra-texture path, or null if the
    // scene has none (or isn't readable as text, e.g. .usdc/.usdz). Cheap: one read at Start.
    private static string? DetectSensorProduct(string scenePath, string? viewportProduct)
    {
        try
        {
            if (string.IsNullOrEmpty(scenePath) || !File.Exists(scenePath)) return null;
            string text = File.ReadAllText(scenePath);
            if (!text.Contains("DistanceToImagePlane", StringComparison.Ordinal)) return null; // no depth authored

            const string prefix = "/Render/OmniverseKit/HydraTextures/";
            string? viewportLeaf = viewportProduct?.Split('/').LastOrDefault();
            foreach (System.Text.RegularExpressions.Match m in
                     System.Text.RegularExpressions.Regex.Matches(text, "def RenderProduct \"([^\"]+)\""))
            {
                string name = m.Groups[1].Value;
                if (name.Contains("ViewportTexture", StringComparison.Ordinal)) continue;
                if (viewportLeaf is not null && name == viewportLeaf) continue;
                return prefix + name; // first non-viewport product in a depth-bearing scene
            }
        }
        catch { /* unreadable scene → no sensor preview */ }
        return null;
    }

    // Locates the repo's scenes/ folder by walking up from the app base directory to the solution.
    private static string? SceneFolder()
    {
        for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
            if (File.Exists(Path.Combine(dir.FullName, "Gemelli.slnx")))
                return Path.Combine(dir.FullName, "scenes");
        return null;
    }

    /// <summary>Opens a file picker and selects the chosen USD scene.</summary>
    private async Task BrowseScene()
    {
        var files = await StorageProvider.OpenFilePickerAsync(new FilePickerOpenOptions
        {
            Title = "Open USD scene",
            AllowMultiple = false,
            FileTypeFilter = [new FilePickerFileType("USD scenes") { Patterns = ["*.usd", "*.usda", "*.usdc", "*.usdz"] }],
        });
        if (files.Count > 0) SelectScene(files[0].Path.LocalPath);
    }

    // Adds a path to the dropdown (if new) and selects it.
    private void SelectScene(string path)
    {
        SceneItem? existing = _scenes.FirstOrDefault(s => string.Equals(s.Path, path, StringComparison.OrdinalIgnoreCase));
        if (existing is null)
        {
            existing = new SceneItem(Path.GetFileName(path), path);
            _scenes.Add(existing);
        }
        _sceneCombo.SelectedItem = existing;
    }

    // ====================================================================== lifecycle

    /// <summary>Starts the twin: resolves native libs, detects sensor/robot, picks the viewport renderer (Fast raster vs RTX), then launches the workers off the UI thread.</summary>
    private void OnStart()
    {
        if (_twin.IsRunning) return;
        _activeProduct = _productBox.Text;

        // A scene may carry a second, fixed render product with depth (e.g. franka_studio's camera_sensor).
        // Detect it and render it alongside the orbit viewport so the SENSOR panel can show depth/segmentation.
        if (string.IsNullOrWhiteSpace(CurrentScenePath)) { _statusLeft.Text = "Pick a scene first (dropdown or Browse)."; return; }

        // Resolve native libraries (env vars override; otherwise auto-discovered under native/). Surface a
        // clear, actionable message instead of a cryptic DLL-load failure if they're missing.
        string? physxLib = GemelliEnvironment.ResolveOvPhysxLibrary();
        string? ovrtxDir = GemelliEnvironment.ResolveOvrtxDirectory();
        if (physxLib is null || ovrtxDir is null)
        {
            _statusLeft.Text = "Native libs not found. Place ovphysx.dll under native/ovphysx/ovphysx/lib and " +
                               "ovrtx-dynamic.dll under native/ovrtx/bin, or set OVPHYSX_LIB / GEMELLI_OVRTX_DIR.";
            return;
        }

        _sensorProduct = DetectSensorProduct(CurrentScenePath, _activeProduct);
        ReadCameraParams(CurrentScenePath);

        // Fast (rasterized) viewport: ovrtx renders only the sensor product (or nothing if there is none);
        // the GL rasterizer drives the main viewport. RTX: ovrtx renders the viewport (+ sensor) as before.
        bool fast = (_viewportModeBox.SelectedItem as string) != "RTX";
        string[] products;
        bool renderEnabled = true;
        if (fast)
        {
            if (_sensorProduct is not null) products = [_sensorProduct];
            else { products = [_activeProduct ?? ""]; renderEnabled = false; } // no sensor → ovrtx idle, sim only
        }
        else
        {
            products = _sensorProduct is null ? [_activeProduct ?? ""] : [_activeProduct ?? "", _sensorProduct];
        }

        var options = new SimulationOptions
        {
            UsdPath = EffectiveScenePath(),
            RenderProducts = products,
            RenderEnabled = renderEnabled,
            Device = (_deviceBox.SelectedItem as string) switch { "gpu" => PhysicsDevice.Gpu, "auto" => PhysicsDevice.Auto, _ => PhysicsDevice.Cpu },
            OvPhysxLibrary = physxLib,
            OvrtxLibraryDirectory = ovrtxDir,
            TimeStep = (float)(_timeStepMs / 1000.0),
            TimeScale = (float)_timeScale,
        };

        _statusLeft.Text = "Starting twin (launching workers, compiling shaders on first run)…";
        _startBtn.IsEnabled = false;

        Task.Run(() =>
        {
            try
            {
                _twin.Start(options);
                // Render the sensor camera (depth/seg) at a quarter rate so it doesn't halve the viewport fps.
                if (_sensorProduct is not null) _twin.SetSecondaryRenderInterval(4);
                DetectRobot();
                Dispatcher.UIThread.Post(() =>
                {
                    _outliner.ItemsSource = _twin.RigidBodyPaths.ToArray();
                    _viewportHint.IsVisible = false;
                    _sensorHint.IsVisible = _sensorProduct is null;
                    _recordBtn.IsEnabled = _sensorProduct is not null;
                    SetRunningControls(true);
                    AutoFrame();
                    if (fast) StartRasterViewport();
                    _twin.Play();
                });
            }
            catch (Exception ex)
            {
                Dispatcher.UIThread.Post(() => { _statusLeft.Text = "Start failed: " + ex.Message.Split('\n')[0]; _startBtn.IsEnabled = true; });
            }
        });
    }

    /// <summary>Stops the twin and tears down recording, raster viewport, robot/IK, and selection state.</summary>
    private void OnStop()
    {
        StopRecording();
        StopRasterViewport();
        Task.Run(() =>
        {
            _twin.Stop();
            _ikController = null; _robotPath = null;
            Dispatcher.UIThread.Post(() => { SetRunningControls(false); _startBtn.IsEnabled = true; _statusLeft.Text = "Stopped."; _viewportHint.IsVisible = true; _inspectedPath = null; _gizmoCanvas.Children.Clear(); });
        });
    }

    /// <summary>Disposes any active sensor recorder and resets the record button.</summary>
    private void StopRecording()
    {
        if (_recorder is null) return;
        _recorder.Dispose();
        _recorder = null;
        _recordBtn.Content = "● Rec";
    }

    /// <summary>Twin fault callback: surfaces the error and returns the UI to a stopped state.</summary>
    private void OnFaulted(Exception ex) => Dispatcher.UIThread.Post(() =>
    {
        _statusLeft.Text = "Twin faulted: " + ex.Message.Split('\n')[0];
        SetRunningControls(false);
        _startBtn.IsEnabled = true;
    });

    // ====================================================================== viewport

    // Frame the camera on the scene's rigid bodies (centroid + spread), instead of a fixed default
    // tuned for one scene. Falls back to a neutral near-origin view if there are no rigid bodies.
    private void AutoFrame()
    {
        try
        {
            float[] poses = _twin.Invoke(api => api.Read(SimTensor.RigidBodyPose, "/World/**"));
            int n = poses.Length / 7;
            if (n == 0) { _camera.Frame(0, 0, 0.5f, 4f); PushCamera(); return; }

            float cx = 0, cy = 0, cz = 0;
            for (int i = 0; i < n; i++) { cx += poses[i * 7]; cy += poses[i * 7 + 1]; cz += poses[i * 7 + 2]; }
            cx /= n; cy /= n; cz /= n;

            float radius = 0;
            for (int i = 0; i < n; i++)
            {
                float dx = poses[i * 7] - cx, dy = poses[i * 7 + 1] - cy, dz = poses[i * 7 + 2] - cz;
                radius = Math.Max(radius, MathF.Sqrt(dx * dx + dy * dy + dz * dz));
            }
            float distance = Math.Clamp(radius * 2.5f + 2.5f, 3f, 400f);
            _camera.Frame(cx, cy, cz, distance);
            PushCamera();
        }
        catch { PushCamera(); }
    }

    /// <summary>Pointer-drag camera control: Shift-drag pans, plain drag orbits; pushes the new transform to the renderer.</summary>
    private void OnViewportDrag(Point p, KeyModifiers mods)
    {
        if (!_dragging) return;
        double dx = p.X - _lastPointer.X, dy = p.Y - _lastPointer.Y;
        _lastPointer = p;
        if (mods.HasFlag(KeyModifiers.Shift))
            _camera.Pan((float)dx, (float)dy);
        else
            _camera.Orbit((float)(-dx * 0.01), (float)(dy * 0.01));
        PushCamera();
    }

    /// <summary>Sends the orbit camera's current transform to the running twin's render camera.</summary>
    private void PushCamera()
    {
        if (_twin.IsRunning) _twin.SetCameraTransform(CameraPath, _camera.ToUsdMatrix());
    }

    // Maps the letterboxed render image inside the viewport control: pixel scale + top-left offset + size.
    private (double Scale, double OffX, double OffY, int W, int H)? ImageRect()
    {
        if (_viewportBitmap is null) return null;
        int iw = _viewportBitmap.PixelSize.Width, ih = _viewportBitmap.PixelSize.Height;
        double cw = _viewportHost.Bounds.Width, ch = _viewportHost.Bounds.Height;
        if (iw == 0 || ih == 0 || cw <= 0 || ch <= 0) return null;
        double scale = Math.Min(cw / iw, ch / ih);
        return (scale, (cw - iw * scale) / 2, (ch - ih * scale) / 2, iw, ih);
    }

    // On left-press, find the rigid body whose projected centre is nearest the cursor (within a tolerance)
    // and start dragging it instead of the camera.
    private void TryPickObject(Point p)
    {
        if (!_twin.IsRunning) return;
        if (ImageRect() is not { } r) return;
        double cpx = (p.X - r.OffX) / r.Scale, cpy = (p.Y - r.OffY) / r.Scale;
        if (cpx < 0 || cpy < 0 || cpx >= r.W || cpy >= r.H) return; // clicked the letterbox → camera

        string[] paths = _twin.RigidBodyPaths.ToArray();
        float vfov = ViewportPick.VerticalFov(_camFocal, _camHorizAperture, (float)r.W / r.H);
        double tol = 0.028 * r.H, best = tol * tol; // tighter: pick within ~2.8% of image height of the cursor
        int hit = -1; float hitDist = 0;
        var hitPose = new float[7];
        for (int i = 0; i < paths.Length; i++)
        {
            // Free rigid bodies are teleported; skip any that belong to the articulation (handled by IK).
            if (_robotPath is not null && paths[i].StartsWith(_robotPath, StringComparison.Ordinal)) continue;
            float[]? pose = _twin.TryGetPose(paths[i]);
            if (pose is null || pose.Length < 7) continue;
            var world = new System.Numerics.Vector3(pose[0], pose[1], pose[2]);
            if (!ViewportPick.ProjectToImage(world, _camera, vfov, r.W, r.H, out float px, out float py, out float dist)) continue;
            double d = (px - cpx) * (px - cpx) + (py - cpy) * (py - cpy);
            // Within tolerance, prefer the front-most body (nearer the camera) when two overlap on screen.
            if (d < best || (d < tol * tol && hit >= 0 && dist < hitDist - 0.05f && Math.Abs(d - best) < 64))
            { best = Math.Min(best, d); hit = i; hitDist = dist; hitPose = pose; }
        }

        // Robot articulation links: pick the nearest link and, if it wins, drive it with IK instead.
        int robotLink = -1; float robotDist = 0; var robotPos = new System.Numerics.Vector3();
        if (_robotPath is not null && _robotLinkCount > 0)
        {
            float[] links;
            try { links = _twin.Invoke(api => api.ReadShaped(SimTensor.ArticulationLinkPose, _robotPath).Data); }
            catch { links = []; }
            int lc = links.Length / 7;
            for (int i = 0; i < lc; i++)
            {
                var world = new System.Numerics.Vector3(links[i * 7], links[i * 7 + 1], links[i * 7 + 2]);
                if (!ViewportPick.ProjectToImage(world, _camera, vfov, r.W, r.H, out float px, out float py, out float dist)) continue;
                double d = (px - cpx) * (px - cpx) + (py - cpy) * (py - cpy);
                if (d < best) { best = d; robotLink = i; robotDist = dist; robotPos = world; hit = -1; }
            }
        }

        if (robotLink >= 0)
        {
            // Grab a robot link → install an IK controller that drives it toward the cursor target.
            _objectDrag = true;
            _dragPath = $"{_robotPath} · link[{robotLink}]";
            _dragDist = robotDist;
            _dragTarget = robotPos;
            _ikController = new IkDragController(_robotPath!, robotLink, robotPos.X, robotPos.Y, robotPos.Z);
            _twin.SetLiveController(_ikController);
            _statusLeft.Text = $"Grabbed robot link[{robotLink}] — drag to pose (IK)";
            return;
        }
        if (hit < 0) return;

        _objectDrag = true;
        _ikController = null;
        _dragPath = paths[hit];
        _dragDist = hitDist;
        _dragTarget = new System.Numerics.Vector3(hitPose[0], hitPose[1], hitPose[2]);
        _dragQuat = [hitPose[3], hitPose[4], hitPose[5], hitPose[6]];

        // Set up ground-plane dragging: the body slides on the horizontal plane through its centre, and we
        // record where on that plane the cursor first grabbed it so it doesn't jump under the cursor.
        _dragPlaneZ = _dragTarget.Z;
        _dragGroundOffset = System.Numerics.Vector3.Zero;
        if (ViewportPick.GroundPoint(_camera, vfov, r.W, r.H, (float)cpx, (float)cpy, _dragPlaneZ) is { } g)
            _dragGroundOffset = new System.Numerics.Vector3(_dragTarget.X - g.X, _dragTarget.Y - g.Y, 0);

        _outliner.SelectedItem = _dragPath;
        _statusLeft.Text = $"Grabbed {_dragPath} — drag to slide on the floor · use the gizmo axes for precise/vertical";
    }

    // Detects an articulation (robot) so the viewport can IK-drag it: reads the root from the scene's
    // PhysicsArticulationRootAPI prim and confirms link poses are available.
    private void DetectRobot()
    {
        _robotPath = null; _robotLinkCount = 0;
        try
        {
            string text = File.ReadAllText(CurrentScenePath);
            var m = System.Text.RegularExpressions.Regex.Match(text, "def \\w+ \"(\\w+)\"\\s*\\([^{]*ArticulationRootAPI");
            if (!m.Success) return;
            string path = "/World/" + m.Groups[1].Value;
            float[] links = _twin.Invoke(api => api.ReadShaped(SimTensor.ArticulationLinkPose, path).Data);
            if (links.Length >= 7) { _robotPath = path; _robotLinkCount = links.Length / 7; }
        }
        catch { /* no robot → drag only handles rigid bodies */ }
    }

    // ====================================================================== transform gizmo

    // Projects a world point to viewport-control coordinates (accounts for letterboxing); null if off-screen.
    private (Point P, float Dist)? ProjectToControl(System.Numerics.Vector3 world)
    {
        if (ImageRect() is not { } r) return null;
        float vfov = ViewportPick.VerticalFov(_camFocal, _camHorizAperture, (float)r.W / r.H);
        if (!ViewportPick.ProjectToImage(world, _camera, vfov, r.W, r.H, out float px, out float py, out float dist)) return null;
        return (new Point(r.OffX + px * r.Scale, r.OffY + py * r.Scale), dist);
    }

    // The rigid body the gizmo currently targets (the outliner selection), with its live pose; or null.
    private (string Path, System.Numerics.Vector3 Pos, float[] Quat)? GizmoTarget()
    {
        if (!_twin.IsRunning || _outliner.SelectedItem is not string path) return null;
        if (_robotPath is not null && path.StartsWith(_robotPath, StringComparison.Ordinal)) return null; // robot uses IK
        float[]? pose = _twin.TryGetPose(path); // lock-free cache (no blocking Invoke on the UI thread)
        if (pose is null || pose.Length < 7) return null;
        return (path, new System.Numerics.Vector3(pose[0], pose[1], pose[2]), [pose[3], pose[4], pose[5], pose[6]]);
    }

    // World length that renders the gizmo at a roughly constant ~70px on screen, given the pivot's depth.
    private float GizmoWorldLength(float dist)
    {
        if (ImageRect() is not { } r) return 0.2f;
        float vfov = ViewportPick.VerticalFov(_camFocal, _camHorizAperture, (float)r.W / r.H);
        return 70f * (2f * dist * MathF.Tan(vfov * 0.5f) / MathF.Max(1, r.H));
    }

    // Redraws the 3-axis handles at the selected body's screen position.
    private void UpdateGizmo()
    {
        _gizmoCanvas.Children.Clear();
        if (GizmoTarget() is not { } t) return;
        if (ProjectToControl(t.Pos) is not { } o) return;
        float len = GizmoWorldLength(o.Dist);

        for (int a = 0; a < 3; a++)
        {
            if (ProjectToControl(t.Pos + AxisDirs[a] * len) is not { } e) continue;
            _gizmoCanvas.Children.Add(new Line
            {
                StartPoint = o.P, EndPoint = e.P, Stroke = AxisBrushes[a], StrokeThickness = _gizmoAxis == a ? 4 : 2.5,
            });
            var dot = new Ellipse { Width = 11, Height = 11, Fill = AxisBrushes[a] };
            Canvas.SetLeft(dot, e.P.X - 5.5); Canvas.SetTop(dot, e.P.Y - 5.5);
            _gizmoCanvas.Children.Add(dot);
        }
        var hub = new Ellipse { Width = 7, Height = 7, Fill = Brushes.White };
        Canvas.SetLeft(hub, o.P.X - 3.5); Canvas.SetTop(hub, o.P.Y - 3.5);
        _gizmoCanvas.Children.Add(hub);
    }

    // If the press is near an axis line, begin an axis-constrained drag and return true.
    private bool TryGrabGizmoAxis(Point p)
    {
        if (GizmoTarget() is not { } t || ProjectToControl(t.Pos) is not { } o) return false;
        float len = GizmoWorldLength(o.Dist);
        for (int a = 0; a < 3; a++)
        {
            if (ProjectToControl(t.Pos + AxisDirs[a] * len) is not { } e) continue;
            if (DistToSegment(p, o.P, e.P) > 9) continue;
            _gizmoAxis = a; _objectDrag = true; _dragPath = t.Path;
            _gizmoPivot = t.Pos; _gizmoQuat = t.Quat;
            _statusLeft.Text = $"Gizmo: dragging {t.Path} along {"XYZ"[a]}";
            return true;
        }
        return false;
    }

    // Moves the body along the locked world axis so its screen motion follows the cursor.
    private void DragAlongAxis(Point p)
    {
        if (_dragPath is null) return;
        var axis = AxisDirs[_gizmoAxis];
        if (ProjectToControl(_gizmoPivot) is not { } a0 || ProjectToControl(_gizmoPivot + axis) is not { } a1) { _lastPointer = p; return; }
        double sx = a1.P.X - a0.P.X, sy = a1.P.Y - a0.P.Y; // control px per 1 world unit along the axis
        double denom = sx * sx + sy * sy;
        if (denom < 1e-6) { _lastPointer = p; return; }
        double t = ((p.X - _lastPointer.X) * sx + (p.Y - _lastPointer.Y) * sy) / denom;
        _lastPointer = p;
        _gizmoPivot += axis * (float)t;

        string path = _dragPath;
        float[] pose = [_gizmoPivot.X, _gizmoPivot.Y, _gizmoPivot.Z, _gizmoQuat[0], _gizmoQuat[1], _gizmoQuat[2], _gizmoQuat[3]];
        Task.Run(() => { try { _twin.Invoke(api => api.Write(SimTensor.RigidBodyPose, path, pose)); } catch { } });
        UpdateGizmo();
    }

    /// <summary>Shortest distance from point p to segment a→b (gizmo-axis hit testing).</summary>
    private static double DistToSegment(Point p, Point a, Point b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double len2 = dx * dx + dy * dy;
        double t = len2 < 1e-9 ? 0 : Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / len2, 0, 1);
        double cx = a.X + t * dx, cy = a.Y + t * dy;
        return Math.Sqrt((p.X - cx) * (p.X - cx) + (p.Y - cy) * (p.Y - cy));
    }

    // Move the grabbed body on the camera-facing plane, writing its pose live. Position is held (gravity
    // paused for it during the drag) so it tracks the cursor cleanly; physics resumes on release.
    private void DragObject(Point p)
    {
        if (_dragPath is null || ImageRect() is not { } r) return;
        double dCx = p.X - _lastPointer.X, dCy = p.Y - _lastPointer.Y;
        _lastPointer = p;
        float vfov = ViewportPick.VerticalFov(_camFocal, _camHorizAperture, (float)r.W / r.H);

        if (_ikController is not null)
        {
            // Robot link IK: keep the camera-plane mapping (no fixed ground height for an end-effector).
            var dW = ViewportPick.DragDeltaWorld((float)(dCx / r.Scale), (float)(dCy / r.Scale), _camera, vfov, r.W, r.H, _dragDist);
            _dragTarget += dW;
            _ikController.SetTarget(_dragTarget.X, _dragTarget.Y, _dragTarget.Z);
            _lastPointer = p;
            return;
        }

        // Free body: slide it on the floor plane to the point under the cursor (intuitive, exact tracking).
        double cpx = (p.X - r.OffX) / r.Scale, cpy = (p.Y - r.OffY) / r.Scale;
        if (ViewportPick.GroundPoint(_camera, vfov, r.W, r.H, (float)cpx, (float)cpy, _dragPlaneZ) is { } g)
            _dragTarget = new System.Numerics.Vector3(g.X + _dragGroundOffset.X, g.Y + _dragGroundOffset.Y, _dragPlaneZ);
        else // grazing angle: fall back to camera-plane delta
            _dragTarget += ViewportPick.DragDeltaWorld((float)(dCx / r.Scale), (float)(dCy / r.Scale), _camera, vfov, r.W, r.H, _dragDist);
        _lastPointer = p;

        string path = _dragPath;
        float[] pose = [_dragTarget.X, _dragTarget.Y, _dragTarget.Z, _dragQuat[0], _dragQuat[1], _dragQuat[2], _dragQuat[3]];
        Task.Run(() => { try { _twin.Invoke(api => api.Write(SimTensor.RigidBodyPose, path, pose)); } catch { } });
    }

    // Reads the render camera's focal length + horizontal aperture from the scene so picking matches the
    // rendered projection. Falls back to Kit's default perspective camera values.
    private void ReadCameraParams(string scenePath)
    {
        _camFocal = 18.147562f; _camHorizAperture = 20.955f;
        try
        {
            if (string.IsNullOrEmpty(scenePath) || !File.Exists(scenePath)) return;
            string text = File.ReadAllText(scenePath);
            var f = System.Text.RegularExpressions.Regex.Match(text, @"float focalLength = ([0-9.eE+-]+)");
            if (f.Success) _camFocal = float.Parse(f.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
            var a = System.Text.RegularExpressions.Regex.Match(text, @"float horizontalAperture = ([0-9.eE+-]+)");
            if (a.Success) _camHorizAperture = float.Parse(a.Groups[1].Value, System.Globalization.CultureInfo.InvariantCulture);
        }
        catch { /* keep defaults */ }
    }

    /// <summary>
    /// Twin frame callback (off the UI thread): updates fps, stashes the viewport color, and schedules a
    /// single UI-thread blit; likewise records/redraws the sensor product. Coalesced via the *PostScheduled
    /// flags so a slow UI can't queue a backlog of frames.
    /// </summary>
    private void OnFrameProduced(IReadOnlyList<CapturedFrame> frames)
    {
        CapturedFrame? frame = _activeProduct is null
            ? frames.FirstOrDefault(f => f.Color is not null)
            : frames.FirstOrDefault(f => f.RenderProduct == _activeProduct);
        RenderVarData? color = frame?.Color;
        if (color is null || color.Width == 0 || color.Height == 0) return;

        long now = Stopwatch.GetTimestamp();
        if (_lastFrameTick != 0)
        {
            double dt = (now - _lastFrameTick) / (double)Stopwatch.Frequency;
            if (dt > 0) _fps = 0.9 * _fps + 0.1 * (1.0 / dt);
        }
        _lastFrameTick = now;

        _pendingColor = color;
        if (Interlocked.Exchange(ref _postScheduled, 1) == 0)
            Dispatcher.UIThread.Post(ApplyPendingFrame);

        // Sensor product (depth/segmentation): stash the whole frame, record it, and schedule a redraw.
        if (_sensorProduct is not null)
        {
            CapturedFrame? sensor = frames.FirstOrDefault(f => f.RenderProduct == _sensorProduct);
            if (sensor is not null)
            {
                _recorder?.Submit(Interlocked.Increment(ref _recordIndex), sensor);
                _pendingSensorFrame = sensor;
                if (Interlocked.Exchange(ref _sensorPostScheduled, 1) == 0)
                    Dispatcher.UIThread.Post(ApplyPendingSensor);
            }
        }
    }

    /// <summary>
    /// UI-thread blit of the latest viewport frame into the WriteableBitmap, converting source RGB/RGBA rows
    /// to premultiplied BGRA. (Re)allocates the bitmap when the frame size changes.
    /// </summary>
    private unsafe void ApplyPendingFrame()
    {
        Interlocked.Exchange(ref _postScheduled, 0);
        RenderVarData? color = _pendingColor;
        if (color is null) return;
        int w = color.Width, h = color.Height, ch = color.Channels;
        if (ch is not (3 or 4)) return;

        try
        {
            if (_viewportBitmap is null || _viewportBitmap.PixelSize.Width != w || _viewportBitmap.PixelSize.Height != h)
            {
                _viewportBitmap = new WriteableBitmap(new PixelSize(w, h), new Vector(96, 96), PixelFormat.Bgra8888, AlphaFormat.Premul);
                _viewport.Source = _viewportBitmap;
            }
            ReadOnlySpan<byte> src = color.Bytes;
            using (ILockedFramebuffer fb = _viewportBitmap.Lock())
            {
                byte* dstBase = (byte*)fb.Address;
                int dstStride = fb.RowBytes;
                fixed (byte* sBase = src)
                {
                    for (int y = 0; y < h; y++)
                    {
                        byte* d = dstBase + y * dstStride;
                        byte* s = sBase + y * w * ch;
                        for (int x = 0; x < w; x++)
                        {
                            byte r = s[0], g = s[1], b = s[2];
                            d[0] = b; d[1] = g; d[2] = r; d[3] = ch == 4 ? s[3] : (byte)255;
                            d += 4; s += ch;
                        }
                    }
                }
            }
            _viewport.InvalidateVisual();
            _viewportOverlay.Text = $"{w}×{h}   {_fps:F0} fps";
        }
        catch { /* skip a bad frame */ }
    }

    // ====================================================================== fast rasterized viewport

    /// <summary>Spins up the GL rasterized viewport, feeding it scene/body data and the camera snapshot, and wiring frame/fault/loaded callbacks.</summary>
    private void StartRasterViewport()
    {
        try
        {
            _raster = new RasterViewport(CurrentScenePath, _twin.RigidBodyPaths.ToArray(), _twin.TryGetPose, CameraSnapshotForRaster);
            _raster.FrameReady += OnRasterFrame;
            _raster.Faulted += ex => Dispatcher.UIThread.Post(() => _statusLeft.Text = "Raster viewport failed: " + ex.Message.Split('\n')[0]);
            _raster.Loaded += (center, radius) => Dispatcher.UIThread.Post(() =>
            {
                _camera.Frame(center.X, center.Y, center.Z, Math.Clamp(radius * 2.0f + 0.3f, 0.6f, 400f));
                PushCamera();
            });
        }
        catch (Exception ex) { _statusLeft.Text = "Raster init failed: " + ex.Message.Split('\n')[0]; }
    }

    /// <summary>Detaches and disposes the raster viewport, joining its thread off the UI thread.</summary>
    private void StopRasterViewport()
    {
        RasterViewport? r = _raster;
        _raster = null;
        if (r is null) return;
        r.FrameReady -= OnRasterFrame;
        Task.Run(() => r.Dispose()); // Join off the UI thread
    }

    // Read on the raster render thread; concurrent float reads of the orbit camera are benign for a viewport.
    private CameraSnapshot CameraSnapshotForRaster()
    {
        _camera.GetView(out System.Numerics.Vector3 eye, out _, out _, out System.Numerics.Vector3 fwd);
        const int W = 1280, H = 720;
        const float Vfov = 0.73f; // ~42° — a normal navigation-viewport FOV (independent of scene camera params)
        return new CameraSnapshot(eye, fwd, Vfov, W, H);
    }

    /// <summary>Raster render-thread callback: updates fps and schedules the shared viewport blit for the new RGBA frame.</summary>
    private void OnRasterFrame(byte[] rgba, int w, int h)
    {
        long now = Stopwatch.GetTimestamp();
        if (_lastFrameTick != 0)
        {
            double dt = (now - _lastFrameTick) / (double)Stopwatch.Frequency;
            if (dt > 0) _fps = 0.9 * _fps + 0.1 * (1.0 / dt);
        }
        _lastFrameTick = now;

        _pendingColor = new RenderVarData("raster", [h, w], ScalarType.UInt, 8, 4, rgba);
        if (Interlocked.Exchange(ref _postScheduled, 1) == 0)
            Dispatcher.UIThread.Post(ApplyPendingFrame);
    }

    // ====================================================================== sensor preview

    /// <summary>The sensor channel chosen in the mode dropdown (defaulting to depth).</summary>
    private SensorChannel SelectedSensorChannel => (_sensorModeCombo.SelectedItem as string) switch
    {
        "Color" => SensorChannel.Color,
        "Segmentation" => SensorChannel.Segmentation,
        _ => SensorChannel.Depth,
    };

    // Re-render the sensor panel from the last frame (e.g. when the mode dropdown changes mid-pause).
    private void RedrawSensor()
    {
        if (_pendingSensorFrame is not null)
        {
            if (Interlocked.Exchange(ref _sensorPostScheduled, 1) == 0)
                Dispatcher.UIThread.Post(ApplyPendingSensor);
        }
    }

    /// <summary>
    /// UI-thread blit of the sensor preview: visualizes the captured frame for the selected channel and
    /// copies it (RGBA→BGRA) into the sensor bitmap.
    /// </summary>
    private unsafe void ApplyPendingSensor()
    {
        Interlocked.Exchange(ref _sensorPostScheduled, 0);
        CapturedFrame? frame = _pendingSensorFrame;
        if (frame is null) return;

        SensorChannel channel = SelectedSensorChannel;
        var img = SensorVisualize.Render(frame, channel);
        if (img is null)
        {
            _sensorOverlay.Text = $"{channel}: not in product";
            return;
        }
        var (w, h, rgba) = img.Value;

        try
        {
            if (_sensorBitmap is null || _sensorBitmap.PixelSize.Width != w || _sensorBitmap.PixelSize.Height != h)
            {
                _sensorBitmap = new WriteableBitmap(new PixelSize(w, h), new Vector(96, 96), PixelFormat.Bgra8888, AlphaFormat.Premul);
                _sensorImage.Source = _sensorBitmap;
            }
            using (ILockedFramebuffer fb = _sensorBitmap.Lock())
            {
                byte* dstBase = (byte*)fb.Address;
                int dstStride = fb.RowBytes;
                fixed (byte* sBase = rgba)
                {
                    for (int y = 0; y < h; y++)
                    {
                        byte* d = dstBase + y * dstStride;
                        byte* s = sBase + y * w * 4;
                        for (int x = 0; x < w; x++)
                        {
                            d[0] = s[2]; d[1] = s[1]; d[2] = s[0]; d[3] = s[3]; // RGBA -> BGRA
                            d += 4; s += 4;
                        }
                    }
                }
            }
            _sensorImage.InvalidateVisual();
            string rec = _recorder is not null ? $"  ● REC {_recorder.Written}" : "";
            _sensorOverlay.Text = $"{channel}  {w}×{h}{rec}";
        }
        catch { /* skip a bad frame */ }
    }

    /// <summary>Starts or stops recording the sensor product to a frame-indexed folder under recordings/.</summary>
    private void ToggleRecording()
    {
        if (_recorder is null)
        {
            if (_sensorProduct is null) return;
            string root = Path.Combine(AppContext.BaseDirectory, "recordings");
            string sceneName = Path.GetFileNameWithoutExtension(CurrentScenePath);
            // Frame count makes the folder name unique and ordered without needing a wall clock.
            string dir = Path.Combine(root, $"{sceneName}_f{_twin.FrameCount}");
            _recorder = new SensorRecorder(dir);
            _recordIndex = 0;
            _recordBtn.Content = "■ Stop";
            _statusLeft.Text = $"Recording → {dir}";
        }
        else
        {
            int n = _recorder.Written, dropped = _recorder.Dropped;
            string dir = _recorder.Directory;
            _recorder.Dispose();
            _recorder = null;
            _recordBtn.Content = "● Rec";
            _statusLeft.Text = $"Recorded {n} frames{(dropped > 0 ? $" ({dropped} dropped)" : "")} → {dir}";
        }
    }

    // ====================================================================== status + inspector

    /// <summary>200ms tick: updates the status/telemetry strip and rebuilds or live-refreshes the inspector for the current selection.</summary>
    private void RefreshStatusAndInspector()
    {
        if (!_twin.IsRunning) return;
        _statusLeft.Text = $"●  {_twin.State}";
        _statusRight.Text = $"t={_twin.SimTime:F2}s   frame {_twin.FrameCount}   {_fps:F0} fps   physics {_twin.LastPhysicsMs:F0}ms   render {_twin.LastRenderMs:F0}ms   bodies {_twin.RigidBodyPaths.Count}";

        if (_outliner.SelectedItem is string path)
        {
            if (path != _inspectedPath) BuildInspector(path);
            else UpdateLiveReadout(path);
        }
    }

    // Build the editable inspector once per selected prim: editable position (X/Y/Z) + Apply, a read-only
    // rotation, and a live readout that the 200ms tick refreshes without destroying the edit fields.
    private void BuildInspector(string path)
    {
        _inspectedPath = path;
        _posX = _posY = _posZ = null;
        _liveReadout = null;

        float[] pose;
        try { pose = _twin.Invoke(api => api.Read(SimTensor.RigidBodyPose, path)); }
        catch { return; }

        _inspector.Children.Clear();
        _inspector.Children.Add(new TextBlock { Text = path, Foreground = Text, FontSize = 13, FontWeight = FontWeight.SemiBold, TextWrapping = TextWrapping.Wrap });

        if (pose.Length < 7)
        {
            _inspector.Children.Add(new TextBlock { Text = "No rigid-body pose for this prim.", Foreground = TextDim, FontSize = 12 });
            return;
        }

        _inspector.Children.Add(Section("Transform (editable)"));
        _posX = NumBox(pose[0]); _posY = NumBox(pose[1]); _posZ = NumBox(pose[2]);
        _inspector.Children.Add(LabeledBoxes("position", _posX, _posY, _posZ));

        var apply = AccentButton("Apply", ApplyPose);
        apply.Padding = new Thickness(12, 4);
        apply.HorizontalAlignment = HorizontalAlignment.Left;
        apply.Margin = new Thickness(0, 4, 0, 0);
        _inspector.Children.Add(apply);

        _inspector.Children.Add(Section("Live state"));
        _liveReadout = new TextBlock { Foreground = TextDim, FontSize = 11, FontFamily = new FontFamily("Consolas, monospace"), TextWrapping = TextWrapping.Wrap };
        _inspector.Children.Add(_liveReadout);
        UpdateLiveReadout(path);
    }

    /// <summary>Refreshes the inspector's live pose readout from the lock-free pose cache, mirroring it into the edit boxes unless the user is typing.</summary>
    private void UpdateLiveReadout(string path)
    {
        if (_liveReadout is null) return;
        float[]? pose = _twin.TryGetPose(path); // lock-free cache, no blocking Invoke
        if (pose is null || pose.Length < 7) return;
        _liveReadout.Text = $"pos  {pose[0]:F3}, {pose[1]:F3}, {pose[2]:F3}\nquat {pose[3]:F3}, {pose[4]:F3}, {pose[5]:F3}, {pose[6]:F3}";

        // Reflect live motion into the edit boxes unless the user is actively editing them.
        if (!_editingPose && _posX is not null && _posY is not null && _posZ is not null)
        {
            _posX.Text = pose[0].ToString("F3"); _posY.Text = pose[1].ToString("F3"); _posZ.Text = pose[2].ToString("F3");
        }
    }

    // Write the typed position back to physics live (keeps the body's current orientation).
    private void ApplyPose()
    {
        if (_inspectedPath is null || _posX is null || _posY is null || _posZ is null) return;
        if (!float.TryParse(_posX.Text, out float x) || !float.TryParse(_posY.Text, out float y) || !float.TryParse(_posZ.Text, out float z))
        { _statusLeft.Text = "Position must be three numbers."; return; }

        string path = _inspectedPath;
        Task.Run(() =>
        {
            try
            {
                _twin.Invoke(api =>
                {
                    float[] pose = api.Read(SimTensor.RigidBodyPose, path);
                    if (pose.Length < 7) return;
                    pose[0] = x; pose[1] = y; pose[2] = z;           // keep quaternion [3..6]
                    api.Write(SimTensor.RigidBodyPose, path, pose);
                });
                Dispatcher.UIThread.Post(() => _statusLeft.Text = $"Moved {path} → ({x:F3}, {y:F3}, {z:F3})");
            }
            catch (Exception ex) { Dispatcher.UIThread.Post(() => _statusLeft.Text = "Edit failed: " + ex.Message.Split('\n')[0]); }
        });
    }

    // Save-back: snapshot every rigid body's current world pose into a standalone USD (via the isolated
    // usd-snapshot tool), so the edited/simulated state becomes the scene's new initial conditions.
    private async Task SaveUsdSnapshot()
    {
        if (!_twin.IsRunning) { _statusLeft.Text = "Start the twin before saving."; return; }
        string scene = CurrentScenePath;
        string? tool = ToolExe("usd-snapshot");
        if (tool is null) { _statusLeft.Text = "usd-snapshot tool not built (tools/usd-snapshot)."; return; }

        var file = await StorageProvider.SaveFilePickerAsync(new FilePickerSaveOptions
        {
            Title = "Save USD snapshot",
            SuggestedFileName = Path.GetFileNameWithoutExtension(scene) + "_snapshot.usda",
            DefaultExtension = "usda",
            FileTypeChoices = [new FilePickerFileType("USD ASCII") { Patterns = ["*.usda"] }],
        });
        if (file is null) return;
        string outPath = file.Path.LocalPath;

        _saveBtn.IsEnabled = false;
        _statusLeft.Text = "Saving snapshot…";
        try
        {
            // Gather current poses (and robot joint state) on the sim thread, then write temp files.
            string[] paths = _twin.RigidBodyPaths.ToArray();
            string? robot = _robotPath;
            string posesFile = Path.GetTempFileName();
            string? jointsFile = null;
            await Task.Run(() =>
            {
                using (var w = new StreamWriter(posesFile))
                    foreach (string p in paths)
                    {
                        float[] pose = _twin.Invoke(api => api.Read(SimTensor.RigidBodyPose, p));
                        if (pose.Length < 7) continue;
                        w.WriteLine($"{p} {pose[0]:R} {pose[1]:R} {pose[2]:R} {pose[3]:R} {pose[4]:R} {pose[5]:R} {pose[6]:R}");
                    }

                if (robot is not null)
                {
                    float[] dofs = _twin.Invoke(api => api.Read(SimTensor.ArticulationDofPosition, robot));
                    IReadOnlyList<string> names = _twin.Invoke(api => api.DofNames(robot));
                    int n = Math.Min(dofs.Length, names.Count);
                    if (n > 0)
                    {
                        jointsFile = Path.GetTempFileName();
                        using var jw = new StreamWriter(jointsFile);
                        for (int i = 0; i < n; i++) jw.WriteLine($"{names[i]} {dofs[i]:R}");
                    }
                }
            });

            string[] toolArgs = jointsFile is null ? [scene, outPath, posesFile] : [scene, outPath, posesFile, jointsFile];
            (int code, string output) = await RunTool(tool, toolArgs);
            try { File.Delete(posesFile); if (jointsFile is not null) File.Delete(jointsFile); } catch { }
            _statusLeft.Text = code == 0 ? $"Saved → {outPath}  ({output.Split('\n').FirstOrDefault()})" : "Save failed: " + output.Split('\n').LastOrDefault(s => s.Length > 0);
        }
        catch (Exception ex) { _statusLeft.Text = "Save failed: " + ex.Message.Split('\n')[0]; }
        finally { _saveBtn.IsEnabled = _twin.IsRunning; }
    }

    /// <summary>Runs an external tool exe to completion, capturing stdout+stderr and the exit code.</summary>
    private static async Task<(int Code, string Output)> RunTool(string exe, string[] args)
    {
        var psi = new ProcessStartInfo(exe) { RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false, CreateNoWindow = true };
        foreach (string a in args) psi.ArgumentList.Add(a);
        using var proc = Process.Start(psi)!;
        string stdout = await proc.StandardOutput.ReadToEndAsync();
        string stderr = await proc.StandardError.ReadToEndAsync();
        await proc.WaitForExitAsync();
        return (proc.ExitCode, stdout + stderr);
    }

    // Finds a built tool exe under tools/<name>/bin/**/<name>.exe (newest), walking up to the solution.
    private static string? ToolExe(string name)
    {
        for (var dir = new DirectoryInfo(AppContext.BaseDirectory); dir is not null; dir = dir.Parent)
        {
            if (!File.Exists(Path.Combine(dir.FullName, "Gemelli.slnx"))) continue;
            string toolBin = Path.Combine(dir.FullName, "tools", name, "bin");
            if (!Directory.Exists(toolBin)) return null;
            return Directory.GetFiles(toolBin, name + ".exe", SearchOption.AllDirectories)
                .OrderByDescending(File.GetLastWriteTimeUtc).FirstOrDefault();
        }
        return null;
    }

    /// <summary>A narrow monospace numeric box that flags <see cref="_editingPose"/> while focused, so live updates don't clobber typing.</summary>
    private TextBox NumBox(float value)
    {
        var tb = new TextBox { Text = value.ToString("F3"), Width = 72, FontSize = 12, FontFamily = new FontFamily("Consolas, monospace") };
        tb.GotFocus += (_, _) => _editingPose = true;
        tb.LostFocus += (_, _) => _editingPose = false;
        return tb;
    }

    /// <summary>A labelled horizontal row of input boxes (e.g. position X/Y/Z).</summary>
    private Control LabeledBoxes(string label, params Control[] boxes)
    {
        var row = new StackPanel { Orientation = Orientation.Horizontal, Spacing = 4, Margin = new Thickness(0, 2) };
        row.Children.Add(new TextBlock { Text = label, Foreground = TextDim, FontSize = 12, Width = 56, VerticalAlignment = VerticalAlignment.Center });
        foreach (Control b in boxes) row.Children.Add(b);
        return row;
    }

    /// <summary>A small accent section heading.</summary>
    private Control Section(string t) => new TextBlock { Text = t.ToUpperInvariant(), Foreground = Accent, FontSize = 10, FontWeight = FontWeight.SemiBold, Margin = new Thickness(0, 8, 0, 2) };

    /// <summary>A read-only label/value row.</summary>
    private Control Row(string label, string value)
    {
        var g = new Grid { ColumnDefinitions = new ColumnDefinitions("70,*") };
        g.Children.Add(new TextBlock { Text = label, Foreground = TextDim, FontSize = 12 });
        var v = new TextBlock { Text = value, Foreground = Text, FontSize = 12, FontFamily = new FontFamily("Consolas, monospace") };
        Grid.SetColumn(v, 1);
        g.Children.Add(v);
        return g;
    }

    /// <summary>Enables transport/save while running and locks the scene/config pickers; reverses when stopped.</summary>
    private void SetRunningControls(bool running)
    {
        _playBtn.IsEnabled = running;
        _pauseBtn.IsEnabled = running;
        _stepBtn.IsEnabled = running;
        _stopBtn.IsEnabled = running;
        _saveBtn.IsEnabled = running;
        _sceneCombo.IsEnabled = !running;
        _deviceBox.IsEnabled = !running;
        _resBox.IsEnabled = !running;
        _viewportModeBox.IsEnabled = !running;
    }

    // ====================================================================== self-test

    // Diagnostic: draw a magenta cross at each rigid body's PROJECTED screen position onto the rendered
    // color image. If the crosses sit on the actual objects, the projection matches the renderer.
    private byte[]? DiagnosticProjectionPng(string product)
    {
        CapturedFrame? frame = _twin.LatestFrames.FirstOrDefault(f => f.RenderProduct == product);
        RenderVarData? color = frame?.Color;
        if (color is null || color.Width == 0 || color.Channels is not (3 or 4)) return null;
        int w = color.Width, h = color.Height, ch = color.Channels;
        byte[] rgba = new byte[w * h * 4];
        ReadOnlySpan<byte> s = color.Bytes;
        for (int i = 0, p = 0, q = 0; i < w * h; i++, p += ch, q += 4)
        { rgba[q] = s[p]; rgba[q + 1] = s[p + 1]; rgba[q + 2] = s[p + 2]; rgba[q + 3] = 255; }

        float vfov = ViewportPick.VerticalFov(_camFocal, _camHorizAperture, (float)w / h);
        foreach (string path in _twin.RigidBodyPaths)
        {
            float[]? pose = _twin.TryGetPose(path);
            if (pose is null || pose.Length < 7) continue;
            var world = new System.Numerics.Vector3(pose[0], pose[1], pose[2]);
            if (!ViewportPick.ProjectToImage(world, _camera, vfov, w, h, out float px, out float py, out _)) continue;
            Console.Error.WriteLine($"[proj] {path} -> ({px:F0},{py:F0})  world=({pose[0]:F2},{pose[1]:F2},{pose[2]:F2})");
            for (int d = -8; d <= 8; d++)
            {
                Mark(rgba, w, h, (int)px + d, (int)py);
                Mark(rgba, w, h, (int)px, (int)py + d);
            }
        }
        return Gemelli.Core.Imaging.Png.Encode(rgba, w, h, 4);

        static void Mark(byte[] buf, int w, int h, int x, int y)
        {
            if (x < 0 || y < 0 || x >= w || y >= h) return;
            int q = (y * w + x) * 4;
            buf[q] = 255; buf[q + 1] = 0; buf[q + 2] = 255; buf[q + 3] = 255; // magenta
        }
    }

    /// <summary>Headless self-test: loads the scene, starts the twin, and once enough frames render saves the current viewport to PNG and exits (guarded by a watchdog timeout).</summary>
    private void RunSelfTest(SelfTestConfig st)
    {
        SelectScene(st.Usd);
        _productBox.Text = st.Product;
        bool saved = false;
        OnStart();

        var timeout = new DispatcherTimer { Interval = TimeSpan.FromSeconds(180) };
        timeout.Tick += (_, _) => { if (!saved) { Environment.ExitCode = 2; Close(); } };
        timeout.Start();

        var poll = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(200) };
        poll.Tick += (_, _) =>
        {
            if (saved || _twin.FrameCount < st.Frames) return;
            // Save whatever the viewport is currently displaying (raster in Fast mode, ovrtx in RTX mode).
            RenderVarData? cur = _pendingColor;
            if (cur is null || cur.Width == 0 || cur.Channels is not (3 or 4)) return;
            byte[] png = Gemelli.Core.Imaging.Png.Encode(cur.Bytes, cur.Width, cur.Height, cur.Channels);
            saved = true;
            File.WriteAllBytes(st.OutPng, png);
            Console.Error.WriteLine($"[selftest] saved {png.Length} bytes after {_twin.FrameCount} frames -> {st.OutPng}");
            Environment.ExitCode = 0;
            Close();
        };
        poll.Start();
    }
}
