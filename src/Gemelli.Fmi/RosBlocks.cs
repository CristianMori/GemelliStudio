using Mori.RosSharp;

namespace Gemelli.Fmi;

/// <summary>
/// One shared <see cref="RosNode"/> per process, reference-counted by the ROS blocks: the first
/// block registers the node with the master, the last one disposing unregisters it. All blocks in
/// a graph naturally talk to the same master, so one node (one XML-RPC endpoint, one TCPROS
/// listener) serves them all.
/// </summary>
public static class RosHub
{
    private static readonly object Lock = new();
    private static RosNode? _node;
    private static int _refs;

    /// <summary>Acquires the shared node, creating it on first use (masterUri wins only then).</summary>
    public static RosNode Acquire(string? masterUri = null)
    {
        lock (Lock)
        {
            _node ??= new RosNode(masterUri, "/gemelli");
            _refs++;
            return _node;
        }
    }

    public static void Release()
    {
        lock (Lock)
        {
            if (--_refs > 0 || _node is null) return;
            _node.Dispose();
            _node = null;
        }
    }
}

/// <summary>
/// Subscribes to a <c>geometry_msgs/Twist</c> topic (velocity commands, e.g. <c>/cmd_vel</c> from
/// teleop_twist_keyboard or a nav stack) and exposes it as two vector pins. The latest message
/// wins; with no publisher the pins hold zero.
/// </summary>
public sealed class RosTwistSubscriberBlock : ISignalBlock
{
    private readonly string _topic;
    private volatile double[] _latest = new double[6];

    public string DisplayName { get; }
    public IReadOnlyList<BlockPin> InputPins { get; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; } =
    [
        new BlockPin("linear", 3, ["x", "y", "z"]),
        new BlockPin("angular", 3, ["x", "y", "z"]),
    ];

    public RosTwistSubscriberBlock(string topic, string? masterUri = null)
    {
        _topic = topic;
        DisplayName = $"ROS sub  {topic}";
        RosNode node = RosHub.Acquire(masterUri);
        try
        {
            node.Subscribe(topic, RosMessages.TwistType, RosMessages.TwistMd5,
                body => _latest = RosMessages.DecodeTwist(body));
        }
        catch
        {
            RosHub.Release();
            throw;
        }
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        double[] v = _latest;
        return new Dictionary<string, double[]>
        {
            ["linear"] = [v[0], v[1], v[2]],
            ["angular"] = [v[3], v[4], v[5]],
        };
    }

    public void Dispose() => RosHub.Release();
}

/// <summary>
/// Publishes its input pins as <c>sensor_msgs/JointState</c> at a fixed rate. Joint names come
/// from the constructor (they must match the consumer's expectations — e.g. URDF joint names for
/// robot_state_publisher); unnamed use falls back to <c>joint_0…N</c>.
/// </summary>
public sealed class RosJointStatePublisherBlock : ISignalBlock
{
    private readonly RosPublisher _pub;
    private readonly IReadOnlyList<string> _names;
    private readonly double _period;
    private double _accumulator;
    private uint _seq;

    public string DisplayName { get; }
    public IReadOnlyList<BlockPin> InputPins { get; }
    public IReadOnlyList<BlockPin> OutputPins { get; } = [];

    public RosJointStatePublisherBlock(
        string topic, IReadOnlyList<string> jointNames, double rateHz = 50, string? masterUri = null)
    {
        _names = jointNames;
        _period = 1.0 / Math.Max(1, rateHz);
        DisplayName = $"ROS pub  {topic}";
        InputPins =
        [
            new BlockPin("positions", jointNames.Count, jointNames),
            new BlockPin("velocities", jointNames.Count, jointNames),
        ];
        RosNode node = RosHub.Acquire(masterUri);
        try
        {
            _pub = node.Advertise(topic, RosMessages.JointStateType, RosMessages.JointStateMd5);
        }
        catch
        {
            RosHub.Release();
            throw;
        }
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        _accumulator += dt;
        if (_accumulator >= _period && _pub.SubscriberCount > 0)
        {
            _accumulator = Math.Min(_accumulator - _period, _period);
            double[] pos = inputs.GetValueOrDefault("positions") ?? [];
            double[] vel = inputs.GetValueOrDefault("velocities") ?? [];
            _pub.Publish(RosMessages.EncodeJointState(_seq++, time, "", _names,
                Sized(pos, _names.Count), Sized(vel, _names.Count), []));
        }
        else if (_accumulator >= _period)
        {
            _accumulator = 0; // no subscribers: skip the serialization, stay ready
        }
        return new Dictionary<string, double[]>();
    }

    private static ReadOnlySpan<double> Sized(double[] v, int width)
    {
        if (v.Length == width) return v;
        var sized = new double[width];
        Array.Copy(v, sized, Math.Min(v.Length, width));
        return sized;
    }

    public void Dispose() => RosHub.Release();
}

/// <summary>
/// Publishes a rigid body's pose and velocity as <c>nav_msgs/Odometry</c> — wire the scene's
/// <c>fmi:bodyPose</c>/<c>fmi:bodyVelocity</c> vectors in and the ROS side sees standard odometry.
/// </summary>
public sealed class RosOdometryPublisherBlock : ISignalBlock
{
    private readonly RosPublisher _pub;
    private readonly double _period;
    private double _accumulator;
    private uint _seq;

    public string DisplayName { get; }
    public IReadOnlyList<BlockPin> InputPins { get; } =
    [
        new BlockPin("pose", 7, ["px", "py", "pz", "qx", "qy", "qz", "qw"]),
        new BlockPin("velocity", 6, ["vx", "vy", "vz", "wx", "wy", "wz"]),
    ];
    public IReadOnlyList<BlockPin> OutputPins { get; } = [];

    public RosOdometryPublisherBlock(string topic, double rateHz = 50, string? masterUri = null)
    {
        _period = 1.0 / Math.Max(1, rateHz);
        DisplayName = $"ROS pub  {topic}";
        RosNode node = RosHub.Acquire(masterUri);
        try
        {
            _pub = node.Advertise(topic, RosMessages.OdometryType, RosMessages.OdometryMd5);
        }
        catch
        {
            RosHub.Release();
            throw;
        }
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        _accumulator += dt;
        if (_accumulator >= _period)
        {
            _accumulator = Math.Min(_accumulator - _period, _period);
            if (_pub.SubscriberCount > 0)
            {
                double[] pose = inputs.GetValueOrDefault("pose") ?? new double[7];
                double[] vel = inputs.GetValueOrDefault("velocity") ?? new double[6];
                if (pose.Length >= 7 && vel.Length >= 6)
                    _pub.Publish(RosMessages.EncodeOdometry(_seq++, time, "odom", "base_link", pose, vel));
            }
        }
        return new Dictionary<string, double[]>();
    }

    public void Dispose() => RosHub.Release();
}

/// <summary>
/// Publishes sim time on <c>/clock</c> every step, so ROS nodes running with
/// <c>use_sim_time = true</c> follow the twin's clock (including Gemelli's time-scale).
/// </summary>
public sealed class RosClockBlock : ISignalBlock
{
    private readonly RosPublisher _pub;

    public string DisplayName => "ROS /clock";
    public IReadOnlyList<BlockPin> InputPins { get; } = [];
    public IReadOnlyList<BlockPin> OutputPins { get; } = [];

    public RosClockBlock(string? masterUri = null)
    {
        RosNode node = RosHub.Acquire(masterUri);
        try
        {
            _pub = node.Advertise("/clock", RosMessages.ClockType, RosMessages.ClockMd5);
        }
        catch
        {
            RosHub.Release();
            throw;
        }
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) { }

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        if (_pub.SubscriberCount > 0) _pub.Publish(RosMessages.EncodeClock(time));
        return new Dictionary<string, double[]>();
    }

    public void Dispose() => RosHub.Release();
}
