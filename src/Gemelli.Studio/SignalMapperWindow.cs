using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Shapes;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Threading;
using Gemelli.Fmi;

namespace Gemelli.Studio;

/// <summary>
/// The signal mapper: a node graph of the running signal graph. Scene sources (sensors, operator
/// values) and source blocks sit on the left, behavior blocks (FMU/SSP, policies) in the middle,
/// actuators on the right. Wires are cubic splines carrying the live value that crossed them on the
/// latest frame; vector wires draw thick with an element count. Nodes drag by their title bar;
/// right-click cuts a wire (or removes a constant/device node); dragging between port dots
/// connects — width mismatches open an element picker on drop. Everything applies to the live
/// <see cref="SignalGraphController"/>, so rewiring takes effect on the next frame.
/// </summary>
public sealed class SignalMapperWindow : Window
{
    private static readonly IBrush Bg = B("#14161B");
    private static readonly IBrush NodeBg = B("#1B1E25");
    private static readonly IBrush NodeHeader = B("#22262E");
    private static readonly IBrush BorderBr = B("#2B313B");
    private static readonly IBrush Text = B("#D7DBE2");
    private static readonly IBrush TextDim = B("#828B99");
    private static readonly IBrush Accent = B("#2EC4B6");
    private static readonly IBrush WireIn = B("#4C8DFF");
    private static readonly IBrush WireOut = B("#E5A34B");
    private static IBrush B(string hex) => new SolidColorBrush(Color.Parse(hex));

    private enum PortKind { SceneSource, BlockInput, BlockOutput, Actuator, Constant }

    /// <summary>A connection point: its node, kind, and either a scene endpoint or a block pin.</summary>
    private sealed class Port
    {
        public required Node Node;
        public required PortKind Kind;
        public required string Label;
        public SignalEndpoint? Endpoint;      // SceneSource / Actuator / Constant ports
        public PinRef? Pin;                   // BlockInput / BlockOutput ports
        public int Width = 1;                 // >1 = vector pin (thick wires)
        public IReadOnlyList<string>? ElementLabels;
        public int RowIndex;                  // vertical slot within the node
        public double LocalY = double.NaN;    // explicit anchor for irregular rows (constant nodes)

        public bool IsRightSide => Kind is PortKind.SceneSource or PortKind.BlockOutput or PortKind.Constant;
        public Point Center => new(
            Node.Pos.X + (IsRightSide ? Node.Width : 0),
            Node.Pos.Y + (double.IsNaN(LocalY) ? HeaderH + RowIndex * RowH + RowH * 0.5 : LocalY));
    }

    /// <summary>A draggable box on the canvas holding a column of ports. Port anchors are computed
    /// from the node position and fixed row metrics, so wires follow a drag with no re-measuring.</summary>
    private sealed class Node
    {
        public required string Title;
        public required double Width;
        public Point Pos;
        public readonly List<Port> Ports = [];
        public Avalonia.Controls.Border? Visual;
        public FmiConstant? Constant;   // non-null for constant nodes (editable value, removable)
        public string? BlockPath;       // non-null for block nodes; "block:N" ones are removable
        public KeyboardBlock? Keyboard; // non-null for keyboard nodes (has an add-key box)
    }

    private const double HeaderH = 26, RowH = 20;

    private readonly SignalGraphController _graph;
    private readonly Canvas _canvas = new() { Background = Brushes.Transparent };
    private readonly WireLayer _wires;
    private readonly List<Node> _nodes = [];
    private readonly List<Port> _ports = [];
    private readonly DispatcherTimer _timer; // repaints the wire layer so value labels stay live

    // Interaction state.
    private Node? _dragNode;
    private Point _dragOffset;
    private Port? _connectFrom;
    private Point _connectCursor;

    public SignalMapperWindow(SignalGraphController graph)
    {
        _graph = graph;
        Title = "Signal Mapper";
        Width = 1150;
        Height = 640;
        Background = Bg;

        _wires = new WireLayer(this);
        var root = new Panel { MinWidth = 1600, MinHeight = 1100 };
        root.Children.Add(_wires);   // wires under the nodes
        root.Children.Add(_canvas);

        var toolbar = new StackPanel { Orientation = Avalonia.Layout.Orientation.Horizontal, Spacing = 10 };
        toolbar.Children.Add(ToolbarButton("+ Constant", () => AddConstantNode(_graph.AddConstant())));
        toolbar.Children.Add(ToolbarButton("+ Gamepad", () => AddDeviceNode(new GamepadBlock())));
        toolbar.Children.Add(ToolbarButton("+ Keyboard", () => AddDeviceNode(new KeyboardBlock())));
        toolbar.Children.Add(ToolbarButton("+ Multiply", () => AddDeviceNode(new MultiplyBlock())));
        toolbar.Children.Add(ToolbarButton("+ ONNX…", () => _ = AddOnnxBlock()));
        toolbar.Children.Add(ToolbarButton("+ ROS…", () => _ = AddRosBlock()));
        toolbar.Children.Add(new TextBlock
        {
            Text = "drag headers to move · drag a dot to connect · right-click a wire to cut",
            Foreground = TextDim, FontSize = 11, VerticalAlignment = Avalonia.Layout.VerticalAlignment.Center,
        });

        var dock = new DockPanel();
        DockPanel.SetDock(toolbar, Dock.Top);
        dock.Children.Add(toolbar);
        dock.Children.Add(new ScrollViewer
        {
            Content = root,
            HorizontalScrollBarVisibility = Avalonia.Controls.Primitives.ScrollBarVisibility.Auto,
        });
        Content = dock;

        BuildGraph();

        _canvas.PointerMoved += OnPointerMoved;
        _canvas.PointerReleased += OnPointerReleased;
        _canvas.PointerPressed += OnCanvasPressed; // the canvas sits over the wire layer

        _timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(150) };
        _timer.Tick += (_, _) => _wires.InvalidateVisual();
        _timer.Start();
        Closed += (_, _) => _timer.Stop();
    }

    private Button ToolbarButton(string text, Action onClick)
    {
        var b = new Button
        {
            Content = text, FontSize = 12, Foreground = Text, Background = NodeHeader,
            Padding = new Thickness(10, 4), Margin = new Thickness(8, 8, 0, 8),
        };
        b.Click += (_, _) => onClick();
        return b;
    }

    // ---------------------------------------------------------------- graph construction

    // Nodes come from the controller: scene source/actuator endpoints from the wire rows (the scene
    // endpoint universe is fixed; wires between them change), block nodes from the block list.
    private void BuildGraph()
    {
        _nodes.Clear();
        _ports.Clear();
        _canvas.Children.Clear();

        IReadOnlyList<SignalMapping> rows = _graph.Mappings;

        // Left: one node per scene prim used as an input source; a port per distinct endpoint.
        var sourceNodes = new Dictionary<string, Node>();
        foreach (SignalMapping r in rows)
            if (r.SourceEndpoint is { } ep && ep.UsdAttribute != SignalGraphController.ConstantAttribute)
                AddEndpointPort(sourceNodes, ep, PortKind.SceneSource, 230);

        // Right: one node per scene prim used as an actuator target.
        var actuatorNodes = new Dictionary<string, Node>();
        foreach (SignalMapping r in rows)
            if (r.SinkEndpoint is { } ep)
                AddEndpointPort(actuatorNodes, ep, PortKind.Actuator, 250);

        // Middle: every block with its full pin surface.
        var blockNodes = new List<Node>();
        foreach ((string path, ISignalBlock block) in _graph.Blocks)
        {
            Node node = MakeBlockNode(path, block);
            blockNodes.Add(node);
        }

        // Column layout: sources x=30, blocks x=430, actuators x=830; stack each column.
        double y = 30;
        foreach (Node n in sourceNodes.Values) { n.Pos = new Point(30, y); y += NodeHeight(n) + 24; }
        y = 30;
        foreach (Node n in blockNodes) { n.Pos = new Point(430, y); y += NodeHeight(n) + 24; }
        y = 30;
        foreach (Node n in actuatorNodes.Values) { n.Pos = new Point(830, y); y += NodeHeight(n) + 24; }

        foreach (Node n in _nodes) BuildNodeVisual(n);

        // Constant nodes already defined on the controller (e.g. window reopened mid-run).
        foreach (FmiConstant c in _graph.Constants) AddConstantNode(c);
        _wires.InvalidateVisual();
    }

    // Creates the node + ports for one block (visual built separately so layout can place it first).
    private Node MakeBlockNode(string path, ISignalBlock block)
    {
        var node = new Node
        {
            Title = block.DisplayName, Width = 240,
            BlockPath = path, Keyboard = block as KeyboardBlock,
        };
        int i = 0;
        foreach (BlockPin p in block.InputPins)
        {
            var port = new Port
            {
                Node = node, Kind = PortKind.BlockInput,
                Label = p.Width > 1 ? $"{p.Name} [{p.Width}]" : p.Name,
                Pin = new PinRef(path, p.Name), Width = p.Width, ElementLabels = p.ElementLabels, RowIndex = i++,
            };
            node.Ports.Add(port); _ports.Add(port);
        }
        foreach (BlockPin p in block.OutputPins)
        {
            var port = new Port
            {
                Node = node, Kind = PortKind.BlockOutput,
                Label = p.Width > 1 ? $"{p.Name} [{p.Width}]" : p.Name,
                Pin = new PinRef(path, p.Name), Width = p.Width, ElementLabels = p.ElementLabels, RowIndex = i++,
            };
            node.Ports.Add(port); _ports.Add(port);
        }
        _nodes.Add(node);
        return node;
    }

    private void AddEndpointPort(Dictionary<string, Node> byPrim, SignalEndpoint ep, PortKind kind, double width)
    {
        if (!byPrim.TryGetValue(ep.TargetPath, out Node? node))
        {
            node = new Node { Title = ep.TargetPath.TrimEnd('/').Split('/')[^1], Width = width };
            byPrim[ep.TargetPath] = node;
            _nodes.Add(node);
        }
        // A port per distinct (attribute, offset) endpoint. Vector endpoints (whole DOF or body-state
        // vectors) keep their width so reconnecting them makes a full-width wire, not an element pick.
        if (node.Ports.Any(p => Equals(p.Endpoint, ep))) return;
        int pinWidth = Math.Max(1, ep.Count);
        string label = ep.UsdAttribute == FmiSchema.PhysxOverlap ? "presence (overlap)" : ep.Label.Split('.')[^1];
        var port = new Port
        {
            Node = node, Kind = kind,
            Label = pinWidth > 1 ? $"{label} [{pinWidth}]" : label,
            Endpoint = ep, Width = pinWidth, RowIndex = node.Ports.Count,
        };
        node.Ports.Add(port);
        _ports.Add(port);
    }

    private static double NodeHeight(Node n) => HeaderH + n.Ports.Count * RowH + (n.Keyboard is null ? 8 : RowH + 12);

    /// <summary>Creates the node for a constant: an editable value box with one output port.</summary>
    private void AddConstantNode(FmiConstant c)
    {
        int existing = _nodes.Count(n => n.Constant is not null);
        var node = new Node
        {
            Title = c.Name, Width = 150, Constant = c,
            Pos = new Point(220, 30 + existing * 96),
        };
        var port = new Port
        {
            Node = node, Kind = PortKind.Constant, Label = "value",
            Endpoint = SignalGraphController.ConstantEndpoint(c),
            LocalY = HeaderH + 17,
        };
        node.Ports.Add(port);
        _ports.Add(port);
        _nodes.Add(node);
        BuildNodeVisual(node);
        _wires.InvalidateVisual();
    }

    /// <summary>Picks an .onnx file and adds it to the graph as a policy block.</summary>
    private async Task AddOnnxBlock()
    {
        var files = await StorageProvider.OpenFilePickerAsync(new Avalonia.Platform.Storage.FilePickerOpenOptions
        {
            Title = "Add ONNX policy block",
            FileTypeFilter = [new Avalonia.Platform.Storage.FilePickerFileType("ONNX model") { Patterns = ["*.onnx"] }],
        });
        if (files.Count == 0) return;
        try
        {
            AddDeviceNode(new OnnxPolicyBlock(files[0].Path.LocalPath));
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine("[fmi] ONNX block failed: " + ex.Message.Split('\n')[0]);
        }
    }

    /// <summary>The "+ ROS…" dialog: pick a role (subscribe/publish), topic, and master URI.</summary>
    private async Task AddRosBlock()
    {
        string[] kinds = ["Subscribe  Twist (cmd_vel)", "Publish  JointState", "Publish  Odometry", "Publish  /clock"];
        string[] defaultTopics = ["/cmd_vel", "/joint_states", "/odom", "/clock"];

        var master = new TextBox
        {
            Text = Environment.GetEnvironmentVariable("ROS_MASTER_URI") ?? "http://localhost:11311/",
        };
        var kind = new ComboBox { ItemsSource = kinds, SelectedIndex = 0, HorizontalAlignment = Avalonia.Layout.HorizontalAlignment.Stretch };
        var topic = new TextBox { Text = defaultTopics[0] };
        // Joint names for the JointState publisher; prefilled from any block exposing a labelled
        // DOF pin (e.g. the duck policy), so the common case is zero typing.
        var names = new TextBox
        {
            Text = string.Join(",", _graph.Blocks
                .SelectMany(b => b.Block.InputPins)
                .FirstOrDefault(p => p.Name == "dof_pos" && p.ElementLabels is not null)?.ElementLabels ?? []),
            Watermark = "joint names, comma-separated",
        };
        var error = new TextBlock { Foreground = Brushes.IndianRed, FontSize = 11, TextWrapping = TextWrapping.Wrap };
        kind.SelectionChanged += (_, _) =>
        {
            int i = Math.Max(0, kind.SelectedIndex);
            topic.Text = defaultTopics[i];
            names.IsEnabled = i == 1;
        };
        names.IsEnabled = false;

        var dialog = new Window
        {
            Title = "Add ROS block",
            Width = 380, SizeToContent = SizeToContent.Height,
            Background = Bg,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
        };
        var ok = new Button { Content = "Add", HorizontalAlignment = Avalonia.Layout.HorizontalAlignment.Right };
        ok.Click += (_, _) =>
        {
            try
            {
                string uri = master.Text ?? "";
                string top = string.IsNullOrWhiteSpace(topic.Text) ? defaultTopics[kind.SelectedIndex] : topic.Text!;
                ISignalBlock block = kind.SelectedIndex switch
                {
                    0 => new RosTwistSubscriberBlock(top, uri),
                    1 => new RosJointStatePublisherBlock(top, ParseNames(names.Text), masterUri: uri),
                    2 => new RosOdometryPublisherBlock(top, masterUri: uri),
                    _ => new RosClockBlock(uri),
                };
                AddDeviceNode(block);
                dialog.Close();
            }
            catch (Exception ex)
            {
                error.Text = ex.Message.Split('\n')[0]; // e.g. master unreachable
            }
        };

        var form = new StackPanel { Spacing = 6, Margin = new Thickness(12) };
        form.Children.Add(Label("Master URI (roscore)"));
        form.Children.Add(master);
        form.Children.Add(Label("Role"));
        form.Children.Add(kind);
        form.Children.Add(Label("Topic"));
        form.Children.Add(topic);
        form.Children.Add(Label("Joint names (JointState only)"));
        form.Children.Add(names);
        form.Children.Add(error);
        form.Children.Add(ok);
        dialog.Content = form;
        await dialog.ShowDialog(this);
        return;

        TextBlock Label(string text) => new() { Text = text, Foreground = TextDim, FontSize = 11 };

        static string[] ParseNames(string? csv) =>
            (csv ?? "").Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
    }

    /// <summary>Adds a runtime block (device, policy) to the running graph and builds its node.</summary>
    private void AddDeviceNode(ISignalBlock block)
    {
        string path = _graph.AddBlock(block);
        int existing = _nodes.Count(n => n.BlockPath is not null && n.BlockPath.StartsWith("block:", StringComparison.Ordinal));
        Node node = MakeBlockNode(path, block);
        node.Pos = new Point(30, 460 + existing * 200); // below the scene sources; drag to taste
        BuildNodeVisual(node);
        _wires.InvalidateVisual();
    }

    private void BuildNodeVisual(Node node)
    {
        var stack = new StackPanel();
        var header = new Avalonia.Controls.Border
        {
            Background = NodeHeader, Height = HeaderH,
            Cursor = new Cursor(StandardCursorType.SizeAll),
            Child = new TextBlock
            {
                Text = node.Title, Foreground = Text, FontSize = 12, FontWeight = FontWeight.SemiBold,
                Margin = new Thickness(8, 5, 8, 0),
            },
        };
        // Nodes drag by their title bar; right-click removes the removable kinds (constants and
        // runtime device blocks) together with their wires.
        header.PointerPressed += (_, e) =>
        {
            if (e.GetCurrentPoint(header).Properties.IsRightButtonPressed && IsRemovable(node))
            {
                RemoveNode(node);
                e.Handled = true;
                return;
            }
            if (!e.GetCurrentPoint(header).Properties.IsLeftButtonPressed) return;
            _dragNode = node;
            _dragOffset = e.GetPosition(_canvas) - node.Pos;
            e.Handled = true;
        };
        stack.Children.Add(header);

        if (node.Constant is { } c)
        {
            // Body: [ value box ][ output dot ] — edits apply on the next frame.
            var row = new DockPanel { Height = 34, Margin = new Thickness(6, 0) };
            var dot = PortDot(node.Ports[0], WireOut);
            DockPanel.SetDock(dot, Dock.Right);
            row.Children.Add(dot);
            var box = new TextBox
            {
                Text = c.Value.ToString("0.###", System.Globalization.CultureInfo.InvariantCulture),
                FontSize = 12, Margin = new Thickness(0, 4, 6, 4), Padding = new Thickness(6, 2),
            };
            box.TextChanged += (_, _) =>
            {
                if (double.TryParse(box.Text, System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out double v))
                    c.Value = v;
            };
            row.Children.Add(box);
            stack.Children.Add(row);
        }
        else
        {
            foreach (Port p in node.Ports)
            {
                bool left = !p.IsRightSide;
                var row = new DockPanel { Height = RowH, Margin = new Thickness(6, 0) };
                var dot = PortDot(p, p.Kind is PortKind.SceneSource or PortKind.BlockInput ? WireIn : WireOut);
                DockPanel.SetDock(dot, left ? Dock.Left : Dock.Right);
                row.Children.Add(dot);
                row.Children.Add(new TextBlock
                {
                    Text = p.Label, Foreground = TextDim, FontSize = 11,
                    Margin = new Thickness(6, 0),
                    VerticalAlignment = Avalonia.Layout.VerticalAlignment.Center,
                    TextAlignment = left ? TextAlignment.Left : TextAlignment.Right,
                });
                stack.Children.Add(row);
            }

            if (node.Keyboard is { } kb)
            {
                // On-the-fly key pins: type a key name (W, Space, Left) and press Enter.
                var addRow = new DockPanel { Height = RowH + 8, Margin = new Thickness(6, 2) };
                var box = new TextBox
                {
                    Watermark = "+ key", FontSize = 11, Padding = new Thickness(6, 2),
                };
                box.KeyDown += (_, e) =>
                {
                    if (e.Key != Key.Enter) return;
                    if (kb.AddKey(box.Text ?? "")) RebuildNodeVisual(node, kb);
                    box.Text = "";
                    e.Handled = true;
                };
                addRow.Children.Add(box);
                stack.Children.Add(addRow);
            }
        }

        var border = new Avalonia.Controls.Border
        {
            Background = NodeBg, BorderBrush = node.Constant is null ? BorderBr : Accent,
            BorderThickness = new Thickness(1),
            CornerRadius = new CornerRadius(5), Width = node.Width, Child = stack,
        };
        node.Visual = border;
        Canvas.SetLeft(border, node.Pos.X);
        Canvas.SetTop(border, node.Pos.Y);
        _canvas.Children.Add(border);
    }

    // A keyboard node grew a pin: rebuild its ports and visual in place (position preserved).
    private void RebuildNodeVisual(Node node, ISignalBlock block)
    {
        _ports.RemoveAll(p => p.Node == node);
        node.Ports.Clear();
        if (node.Visual is not null) _canvas.Children.Remove(node.Visual);
        _nodes.Remove(node);

        Node fresh = MakeBlockNode(node.BlockPath!, block);
        fresh.Pos = node.Pos;
        BuildNodeVisual(fresh);
        _wires.InvalidateVisual();
    }

    /// <summary>The clickable connection dot for a port; pressing it starts a wire drag.</summary>
    private Ellipse PortDot(Port p, IBrush fill)
    {
        var dot = new Ellipse
        {
            Width = p.Width > 1 ? 13 : 10, Height = p.Width > 1 ? 13 : 10, Fill = fill,
            VerticalAlignment = Avalonia.Layout.VerticalAlignment.Center,
            Cursor = new Cursor(StandardCursorType.Hand),
            Tag = p,
        };
        dot.PointerPressed += OnPortPressed;
        return dot;
    }

    private static bool IsRemovable(Node node) =>
        node.Constant is not null
        || (node.BlockPath is not null && node.BlockPath.StartsWith("block:", StringComparison.Ordinal));

    /// <summary>Deletes a removable node: its wires (via the controller), its ports, and its visual.</summary>
    private void RemoveNode(Node node)
    {
        if (node.Constant is { } c) _graph.RemoveConstant(c.Id);
        else if (node.BlockPath is { } path) _graph.RemoveBlock(path);
        _ports.RemoveAll(p => p.Node == node);
        _nodes.Remove(node);
        if (node.Visual is not null) _canvas.Children.Remove(node.Visual);
        _wires.InvalidateVisual();
    }

    // ---------------------------------------------------------------- interaction

    private void OnPortPressed(object? sender, PointerPressedEventArgs e)
    {
        if (sender is not Ellipse { Tag: Port p }) return;
        _connectFrom = p;
        _connectCursor = e.GetPosition(_canvas);
        e.Handled = true;
    }

    private void OnPointerMoved(object? sender, PointerEventArgs e)
    {
        Point pos = e.GetPosition(_canvas);
        if (_dragNode is not null)
        {
            _dragNode.Pos = pos - _dragOffset;
            Canvas.SetLeft(_dragNode.Visual!, _dragNode.Pos.X);
            Canvas.SetTop(_dragNode.Visual!, _dragNode.Pos.Y);
            _wires.InvalidateVisual();
        }
        else if (_connectFrom is not null)
        {
            _connectCursor = pos;
            _wires.InvalidateVisual();
        }
    }

    private async void OnPointerReleased(object? sender, PointerReleasedEventArgs e)
    {
        if (_connectFrom is not null)
        {
            Port from = _connectFrom;
            _connectFrom = null;
            Port? target = HitPort(e.GetPosition(_canvas), 14);
            if (target is not null) await TryConnect(from, target);
            _wires.InvalidateVisual();
        }
        _dragNode = null;
    }

    // Right-click near a wire cuts it.
    private void OnCanvasPressed(object? sender, PointerPressedEventArgs e)
    {
        if (!e.GetCurrentPoint(_canvas).Properties.IsRightButtonPressed) return;
        Point pos = e.GetPosition(_canvas);
        (SignalMapping Row, double Dist)? best = null;
        foreach (SignalMapping row in _graph.Mappings)
        {
            if (WireGeometry(row) is not { } g) continue;
            double d = DistanceToBezier(pos, g.From, g.C1, g.C2, g.To);
            if (best is null || d < best.Value.Dist) best = (row, d);
        }
        if (best is { Dist: <= 9 })
        {
            _graph.RemoveMapping(best.Value.Row.Id);
            _wires.InvalidateVisual();
            e.Handled = true;
        }
    }

    // Valid wires: any source-side port (scene source, constant, block output) to any sink-side
    // port (block input, actuator). Drag direction doesn't matter. Width mismatches open the
    // element picker to choose which component the wire carries.
    private async Task TryConnect(Port a, Port b)
    {
        (Port from, Port to) = a.IsRightSide ? (a, b) : (b, a);
        if (!from.IsRightSide || to.IsRightSide) return;

        int fromW = Math.Max(1, from.Width), toW = Math.Max(1, to.Width);
        int sourceOffset = 0, sinkOffset = 0;
        int count = Math.Min(fromW, toW);
        if (fromW != toW)
        {
            // Pick the element on the wider side; the wire then carries the narrower width.
            if (fromW > toW)
            {
                int? pick = await PickElement(from, $"{from.Label}: which element feeds {to.Label}?");
                if (pick is null) return;
                sourceOffset = pick.Value;
            }
            else
            {
                int? pick = await PickElement(to, $"{to.Label}: which element does {from.Label} drive?");
                if (pick is null) return;
                sinkOffset = pick.Value;
            }
        }

        _graph.Connect(
            sourceEndpoint: from.Endpoint, sourcePin: from.Pin,
            sinkPin: to.Pin, sinkEndpoint: to.Endpoint,
            sourceOffset: sourceOffset, sinkOffset: sinkOffset, count: count);
    }

    // The drop-time element picker: a small modal listing the vector pin's element labels.
    private async Task<int?> PickElement(Port vectorPort, string prompt)
    {
        var list = new ListBox { Background = NodeBg, Foreground = Text };
        var items = new List<string>();
        for (int i = 0; i < vectorPort.Width; i++)
            items.Add(vectorPort.ElementLabels is { } labels && i < labels.Count ? $"{i}: {labels[i]}" : $"element {i}");
        list.ItemsSource = items;

        var dialog = new Window
        {
            Title = "Pick element",
            Width = 340, Height = Math.Min(420, 90 + items.Count * 26),
            Background = Bg,
            WindowStartupLocation = WindowStartupLocation.CenterOwner,
            Content = new DockPanel
            {
                Children =
                {
                    Dock(new TextBlock
                    {
                        Text = prompt, Foreground = TextDim, FontSize = 12, Margin = new Thickness(10, 8),
                        TextWrapping = TextWrapping.Wrap,
                    }, Avalonia.Controls.Dock.Top),
                    new ScrollViewer { Content = list },
                },
            },
        };
        int? result = null;
        list.SelectionChanged += (_, _) =>
        {
            result = list.SelectedIndex >= 0 ? list.SelectedIndex : null;
            dialog.Close();
        };
        await dialog.ShowDialog(this);
        return result;

        static Control Dock(Control c, Avalonia.Controls.Dock dock)
        {
            DockPanel.SetDock(c, dock);
            return c;
        }
    }

    /// <summary>The nearest port dot within <paramref name="radius"/> px of a point, or null.</summary>
    private Port? HitPort(Point pos, double radius)
    {
        Port? best = null;
        double bestD = radius;
        foreach (Port p in _ports)
        {
            double d = Distance(pos, p.Center);
            if (d < bestD) { bestD = d; best = p; }
        }
        return best;
    }

    // ---------------------------------------------------------------- wires

    // Resolves a wire row to its source and sink ports, or null when either has no port on screen.
    private (Port From, Port To)? WirePorts(SignalMapping row)
    {
        Port? from = row.SourcePin is { } sp
            ? _ports.FirstOrDefault(p => p.Kind == PortKind.BlockOutput && Equals(p.Pin, sp))
            : _ports.FirstOrDefault(p => p.Kind is PortKind.SceneSource or PortKind.Constant && Equals(p.Endpoint, row.SourceEndpoint));
        Port? to = row.SinkPin is { } kp
            ? _ports.FirstOrDefault(p => p.Kind == PortKind.BlockInput && Equals(p.Pin, kp))
            : _ports.FirstOrDefault(p => p.Kind == PortKind.Actuator && Equals(p.Endpoint, row.SinkEndpoint));
        return from is null || to is null ? null : (from, to);
    }

    // A wire's cubic-bezier control points (port anchors with horizontal control handles).
    private (Point From, Point C1, Point C2, Point To)? WireGeometry(SignalMapping row)
    {
        if (WirePorts(row) is not { } ports) return null;
        (Point from, Point to) = (ports.From.Center, ports.To.Center);
        double dx = Math.Max(40, Math.Abs(to.X - from.X) * 0.45);
        return (from, new Point(from.X + dx, from.Y), new Point(to.X - dx, to.Y), to);
    }

    private static Point Bezier(Point p0, Point p1, Point p2, Point p3, double t)
    {
        double u = 1 - t;
        double x = u * u * u * p0.X + 3 * u * u * t * p1.X + 3 * u * t * t * p2.X + t * t * t * p3.X;
        double y = u * u * u * p0.Y + 3 * u * u * t * p1.Y + 3 * u * t * t * p2.Y + t * t * t * p3.Y;
        return new Point(x, y);
    }

    private static double Distance(Point a, Point b) => Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    private static double DistanceToBezier(Point p, Point p0, Point p1, Point p2, Point p3)
    {
        double best = double.MaxValue;
        for (int i = 0; i <= 32; i++)
            best = Math.Min(best, Distance(p, Bezier(p0, p1, p2, p3, i / 32.0)));
        return best;
    }

    /// <summary>Draws every wire (and the in-progress connection) with its live value label.</summary>
    private sealed class WireLayer : Control
    {
        private readonly SignalMapperWindow _w;
        public WireLayer(SignalMapperWindow w) { _w = w; }

        public override void Render(DrawingContext ctx)
        {
            // Fill the layer so it hit-tests for the right-click cut everywhere.
            ctx.FillRectangle(Brushes.Transparent, new Rect(Bounds.Size));

            IReadOnlyList<SignalMapping> mappings = _w._graph.Mappings;

            // Unconnected block output pins still show their live value, right of the dot.
            foreach (Port p in _w._ports)
            {
                if (p.Kind != PortKind.BlockOutput || p.Pin is null) continue;
                if (mappings.Any(r => Equals(r.SourcePin, p.Pin))) continue;
                if (_w._graph.BlockOutputs(p.Pin.BlockPath) is not { } outs
                    || outs.GetValueOrDefault(p.Pin.Pin) is not { Length: > 0 } v) continue;
                DrawLabel(ctx, FormatValue(v, p.Width), p.Center + new Point(10, -7), TextDim, boxed: false);
            }

            foreach (SignalMapping row in mappings)
            {
                if (_w.WireGeometry(row) is not { } g) continue;
                IBrush brush = row.SinkPin is not null ? WireIn : WireOut;
                double thickness = row.Count > 1 ? 3.6 : 1.8;
                DrawWire(ctx, g.From, g.C1, g.C2, g.To, new Pen(brush, thickness));

                Point mid = Bezier(g.From, g.C1, g.C2, g.To, 0.5);
                DrawLabel(ctx, FormatValue(row.LastValues, row.Count), new Point(mid.X, mid.Y - 8), brush, boxed: true);
            }

            if (_w._connectFrom is { } from)
            {
                Point a = from.Center, b = _w._connectCursor;
                double dx = Math.Max(40, Math.Abs(b.X - a.X) * 0.45);
                DrawWire(ctx, a, new Point(a.X + dx, a.Y), new Point(b.X - dx, b.Y), b,
                    new Pen(Accent, 1.5, dashStyle: new DashStyle([4, 3], 0)));
            }
        }

        // Scalars print plainly; vectors print "[n] first-value".
        private static string FormatValue(double[] v, int width)
        {
            if (v.Length == 0) return "";
            return width > 1 || v.Length > 1 ? $"[{v.Length}] {v[0]:0.##}…" : v[0].ToString("0.##");
        }

        private void DrawLabel(DrawingContext ctx, string text, Point at, IBrush brush, bool boxed)
        {
            if (text.Length == 0) return;
            var label = new FormattedText(
                text, System.Globalization.CultureInfo.InvariantCulture,
                FlowDirection.LeftToRight, Typeface.Default, 11, brush);
            Point origin = boxed ? new Point(at.X - label.Width / 2, at.Y) : at;
            if (boxed)
                ctx.FillRectangle(Bg, new Rect(origin.X - 3, origin.Y, label.Width + 6, 15));
            ctx.DrawText(label, origin);
        }

        private static void DrawWire(DrawingContext ctx, Point p0, Point c1, Point c2, Point p3, IPen pen)
        {
            var geo = new StreamGeometry();
            using (StreamGeometryContext g = geo.Open())
            {
                g.BeginFigure(p0, isFilled: false);
                g.CubicBezierTo(c1, c2, p3);
                g.EndFigure(false);
            }
            ctx.DrawGeometry(null, pen, geo);
        }
    }
}
