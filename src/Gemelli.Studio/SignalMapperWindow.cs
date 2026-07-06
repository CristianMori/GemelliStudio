using Avalonia;
using Avalonia.Controls;
using Avalonia.Controls.Shapes;
using Avalonia.Input;
using Avalonia.Media;
using Avalonia.Threading;
using Gemelli.Fmi;

namespace Gemelli.Studio;

/// <summary>
/// The signal mapper: a node graph of the running FMI wiring. Signal sources (sensors, operator
/// values) sit on the left, FMI instances in the middle, actuators (drive joints, forces) on the
/// right. Wires are cubic splines carrying the live value that crossed them on the latest frame.
/// Nodes drag; right-click cuts a wire; dragging from a port dot to a compatible port reconnects —
/// all applied to the live <see cref="FmiController"/>, so rewiring takes effect on the next frame.
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

    private enum PortKind { Source, FmiInput, FmiOutput, Actuator, Constant }

    /// <summary>A connection point: its node, kind, and either a scene endpoint or an FMI variable.</summary>
    private sealed class Port
    {
        public required Node Node;
        public required PortKind Kind;
        public required string Label;
        public SignalEndpoint? Endpoint;   // Source / Actuator / Constant ports
        public string? FmuVariable;        // FmiInput / FmiOutput ports
        public int RowIndex;               // vertical slot within the node
        public double LocalY = double.NaN; // explicit anchor for irregular rows (constant nodes)
        public Point Center => new(
            Node.Pos.X + (Kind is PortKind.Source or PortKind.FmiOutput or PortKind.Constant ? Node.Width : 0),
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
        public FmiConstant? Constant; // non-null for constant nodes (editable value, removable)
    }

    private const double HeaderH = 26, RowH = 20;

    private readonly FmiController _fmi;
    private readonly Canvas _canvas = new() { Background = Brushes.Transparent };
    private readonly WireLayer _wires;
    private readonly List<Node> _nodes = [];
    private readonly List<Port> _ports = [];
    private readonly Dictionary<Node, string> _fmiNodeInstance = new(); // FMI node -> its prim path
    private readonly DispatcherTimer _timer; // repaints the wire layer so value labels stay live

    // Interaction state.
    private Node? _dragNode;
    private Point _dragOffset;
    private Port? _connectFrom;
    private Point _connectCursor;

    public SignalMapperWindow(FmiController fmi)
    {
        _fmi = fmi;
        Title = "Signal Mapper";
        Width = 1150;
        Height = 640;
        Background = Bg;

        _wires = new WireLayer(this);
        var root = new Panel { MinWidth = 1600, MinHeight = 1100 };
        root.Children.Add(_wires);   // wires under the nodes
        root.Children.Add(_canvas);

        var addConst = new Button
        {
            Content = "+ Constant", FontSize = 12, Foreground = Text, Background = NodeHeader,
            Padding = new Thickness(10, 4), Margin = new Thickness(8),
        };
        addConst.Click += (_, _) => AddConstantNode(_fmi.AddConstant());
        var hint = new TextBlock
        {
            Text = "drag headers to move · drag a dot to connect · right-click a wire to cut",
            Foreground = TextDim, FontSize = 11, VerticalAlignment = Avalonia.Layout.VerticalAlignment.Center,
        };
        var toolbar = new StackPanel { Orientation = Avalonia.Layout.Orientation.Horizontal, Spacing = 10 };
        toolbar.Children.Add(addConst);
        toolbar.Children.Add(hint);

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

    // ---------------------------------------------------------------- graph construction

    // Nodes come from the controller: source/actuator endpoints from the mapping rows (the endpoint
    // universe is fixed; wires between them change), FMI nodes from the live instance ports.
    private void BuildGraph()
    {
        _nodes.Clear();
        _ports.Clear();
        _canvas.Children.Clear();

        IReadOnlyList<SignalMapping> rows = _fmi.Mappings;

        // Left: one node per input-side target prim; a port per distinct endpoint on it. Overlap
        // sensors get their own flavor of label.
        var sourceNodes = new Dictionary<string, Node>();
        foreach (SignalMapping r in rows.Where(r => r.IsInput))
            AddEndpointPort(sourceNodes, r.Endpoint, PortKind.Source, 230);

        // Right: one node per output-side target prim.
        var actuatorNodes = new Dictionary<string, Node>();
        foreach (SignalMapping r in rows.Where(r => !r.IsInput))
            AddEndpointPort(actuatorNodes, r.Endpoint, PortKind.Actuator, 250);

        // Middle: the FMI instances with their full connectable surface.
        var fmiNodes = new List<Node>();
        foreach (FmiInstancePorts inst in _fmi.InstancePorts)
        {
            var node = new Node { Title = $"{(inst.IsSsp ? "SSP" : "FMU")}  {inst.Name}", Width = 240 };
            int i = 0;
            foreach (string v in inst.Inputs)
            {
                var p = new Port { Node = node, Kind = PortKind.FmiInput, Label = v, FmuVariable = v, RowIndex = i++ };
                node.Ports.Add(p); _ports.Add(p);
            }
            foreach (string v in inst.Outputs)
            {
                var p = new Port { Node = node, Kind = PortKind.FmiOutput, Label = v, FmuVariable = v, RowIndex = i++ };
                node.Ports.Add(p); _ports.Add(p);
            }
            node.Pos = new Point(0, 0); // placed below
            fmiNodes.Add(node);
            _nodes.Add(node);
            _fmiNodeInstance[node] = inst.PrimPath;
        }

        // Column layout: sources x=30, FMI x=430, actuators x=830; stack each column.
        double y = 30;
        foreach (Node n in sourceNodes.Values) { n.Pos = new Point(30, y); y += NodeHeight(n) + 24; }
        y = 30;
        foreach (Node n in fmiNodes) { n.Pos = new Point(430, y); y += NodeHeight(n) + 24; }
        y = 30;
        foreach (Node n in actuatorNodes.Values) { n.Pos = new Point(830, y); y += NodeHeight(n) + 24; }

        foreach (Node n in _nodes) BuildNodeVisual(n);

        // Constant nodes already defined on the controller (e.g. window reopened mid-run).
        foreach (FmiConstant c in _fmi.Constants) AddConstantNode(c);
        _wires.InvalidateVisual();
    }

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
            Endpoint = new SignalEndpoint(c.Path, FmiController.ConstantAttribute, 0, 0),
            LocalY = HeaderH + 17,
        };
        node.Ports.Add(port);
        _ports.Add(port);
        _nodes.Add(node);
        BuildNodeVisual(node);
        _wires.InvalidateVisual();
    }

    private void AddEndpointPort(Dictionary<string, Node> byPrim, SignalEndpoint ep, PortKind kind, double width)
    {
        if (!byPrim.TryGetValue(ep.TargetPath, out Node? node))
        {
            node = new Node { Title = ep.TargetPath.TrimEnd('/').Split('/')[^1], Width = width };
            byPrim[ep.TargetPath] = node;
            _nodes.Add(node);
        }
        // A port per distinct (attribute, offset) endpoint.
        if (node.Ports.Any(p => Equals(p.Endpoint, ep))) return;
        var port = new Port
        {
            Node = node, Kind = kind,
            Label = ep.UsdAttribute == FmiSchema.PhysxOverlap ? "presence (overlap)" : ep.Label.Split('.')[^1],
            Endpoint = ep, RowIndex = node.Ports.Count,
        };
        node.Ports.Add(port);
        _ports.Add(port);
    }

    private static double NodeHeight(Node n) => HeaderH + n.Ports.Count * RowH + 8;

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
        // Nodes drag by their title bar; right-click on a constant's header removes it (and its wires).
        header.PointerPressed += (_, e) =>
        {
            if (e.GetCurrentPoint(header).Properties.IsRightButtonPressed && node.Constant is not null)
            {
                RemoveConstantNode(node);
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
            Port p = node.Ports[0];
            var dot = PortDot(p, WireOut);
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
                bool left = p.Kind is PortKind.FmiInput or PortKind.Actuator;
                var row = new DockPanel { Height = RowH, Margin = new Thickness(6, 0) };
                var dot = PortDot(p, p.Kind is PortKind.Source or PortKind.FmiInput ? WireIn : WireOut);
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

    /// <summary>The clickable connection dot for a port; pressing it starts a wire drag.</summary>
    private Ellipse PortDot(Port p, IBrush fill)
    {
        var dot = new Ellipse
        {
            Width = 10, Height = 10, Fill = fill,
            VerticalAlignment = Avalonia.Layout.VerticalAlignment.Center,
            Cursor = new Cursor(StandardCursorType.Hand),
            Tag = p,
        };
        dot.PointerPressed += OnPortPressed;
        return dot;
    }

    /// <summary>Deletes a constant: its wires (via the controller), its ports, and its visual.</summary>
    private void RemoveConstantNode(Node node)
    {
        if (node.Constant is null) return;
        _fmi.RemoveConstant(node.Constant.Id);
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

    private void OnPointerReleased(object? sender, PointerReleasedEventArgs e)
    {
        if (_connectFrom is not null)
        {
            Port? target = HitPort(e.GetPosition(_canvas), 14);
            if (target is not null) TryConnect(_connectFrom, target);
            _connectFrom = null;
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
        foreach (SignalMapping row in _fmi.Mappings)
        {
            if (WireGeometry(row) is not { } g) continue;
            double d = DistanceToBezier(pos, g.From, g.C1, g.C2, g.To);
            if (best is null || d < best.Value.Dist) best = (row, d);
        }
        if (best is { Dist: <= 9 })
        {
            _fmi.RemoveMapping(best.Value.Row.Id);
            _wires.InvalidateVisual();
            e.Handled = true;
        }
    }

    // Valid wires: source → FMI input, FMI output → actuator, constant → FMI input,
    // constant → actuator (bypasses the models). Direction of the drag doesn't matter.
    private void TryConnect(Port a, Port b)
    {
        (Port from, Port to) = a.Kind is PortKind.Source or PortKind.FmiOutput or PortKind.Constant ? (a, b) : (b, a);
        FmiConstant? c = from.Node.Constant;
        if (from.Kind == PortKind.Source && to.Kind == PortKind.FmiInput)
            _fmi.AddMapping(FindInstancePath(to), to.FmuVariable!, isInput: true, from.Endpoint!);
        else if (from.Kind == PortKind.FmiOutput && to.Kind == PortKind.Actuator)
            _fmi.AddMapping(FindInstancePath(from), from.FmuVariable!, isInput: false, to.Endpoint!);
        else if (from.Kind == PortKind.Constant && to.Kind == PortKind.FmiInput && c is not null)
            _fmi.ConnectConstantToInput(c, FindInstancePath(to), to.FmuVariable!);
        else if (from.Kind == PortKind.Constant && to.Kind == PortKind.Actuator && c is not null)
            _fmi.ConnectConstantToActuator(c, to.Endpoint!);
    }

    private string FindInstancePath(Port fmiPort) =>
        _fmiNodeInstance.TryGetValue(fmiPort.Node, out string? path) ? path : "";

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

    // Resolves a mapping row to its wire's cubic-bezier control points (source and sink port
    // anchors, with horizontal control handles), or null when either endpoint has no port.
    private (Point From, Point C1, Point C2, Point To)? WireGeometry(SignalMapping row)
    {
        Point from, to;
        if (!row.IsInput && row.InstancePath.StartsWith("const:", StringComparison.Ordinal))
        {
            // Constant → actuator: no FMI port involved.
            Port? cPort = _ports.FirstOrDefault(p => p.Kind == PortKind.Constant && p.Endpoint?.TargetPath == row.InstancePath);
            Port? aPort = _ports.FirstOrDefault(p => p.Kind == PortKind.Actuator && Equals(p.Endpoint, row.Endpoint));
            if (cPort is null || aPort is null) return null;
            (from, to) = (cPort.Center, aPort.Center);
        }
        else
        {
            Port? fmiPort = _ports.FirstOrDefault(p =>
                (row.IsInput ? p.Kind == PortKind.FmiInput : p.Kind == PortKind.FmiOutput)
                && p.FmuVariable == row.FmuVariable);
            Port? scenePort = row.IsInput && row.Endpoint.UsdAttribute == FmiController.ConstantAttribute
                ? _ports.FirstOrDefault(p => p.Kind == PortKind.Constant && p.Endpoint?.TargetPath == row.Endpoint.TargetPath)
                : _ports.FirstOrDefault(p =>
                    (row.IsInput ? p.Kind == PortKind.Source : p.Kind == PortKind.Actuator)
                    && Equals(p.Endpoint, row.Endpoint));
            if (fmiPort is null || scenePort is null) return null;
            (from, to) = row.IsInput ? (scenePort.Center, fmiPort.Center) : (fmiPort.Center, scenePort.Center);
        }
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

            IReadOnlyList<SignalMapping> mappings = _w._fmi.Mappings;

            // Unconnected FMI output pins still show their live value, right of the dot.
            foreach (Port p in _w._ports)
            {
                if (p.Kind != PortKind.FmiOutput) continue;
                if (mappings.Any(r => !r.IsInput && r.FmuVariable == p.FmuVariable
                        && r.InstancePath == _w.FindInstancePath(p))) continue;
                if (_w._fmi.InstanceOutputs(_w.FindInstancePath(p)) is not { } outs
                    || !outs.TryGetValue(p.FmuVariable!, out double value)) continue;
                var txt = new FormattedText(
                    value.ToString("0.##"),
                    System.Globalization.CultureInfo.InvariantCulture,
                    FlowDirection.LeftToRight, Typeface.Default, 11, TextDim);
                ctx.DrawText(txt, p.Center + new Point(10, -7));
            }

            foreach (SignalMapping row in mappings)
            {
                if (_w.WireGeometry(row) is not { } g) continue;
                IBrush brush = row.IsInput ? WireIn : WireOut;
                DrawWire(ctx, g.From, g.C1, g.C2, g.To, new Pen(brush, 1.8));

                Point mid = Bezier(g.From, g.C1, g.C2, g.To, 0.5);
                var label = new FormattedText(
                    row.LastValue.ToString("0.##"),
                    System.Globalization.CultureInfo.InvariantCulture,
                    FlowDirection.LeftToRight, Typeface.Default, 11, brush);
                ctx.FillRectangle(Bg, new Rect(mid.X - label.Width / 2 - 3, mid.Y - 8, label.Width + 6, 15));
                ctx.DrawText(label, new Point(mid.X - label.Width / 2, mid.Y - 8));
            }

            if (_w._connectFrom is { } from)
            {
                Point a = from.Center, b = _w._connectCursor;
                double dx = Math.Max(40, Math.Abs(b.X - a.X) * 0.45);
                DrawWire(ctx, a, new Point(a.X + dx, a.Y), new Point(b.X - dx, b.Y), b,
                    new Pen(Accent, 1.5, dashStyle: new DashStyle([4, 3], 0)));
            }
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
