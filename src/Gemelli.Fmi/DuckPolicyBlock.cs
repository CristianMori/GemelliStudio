namespace Gemelli.Fmi;

/// <summary>
/// The Open Duck Mini v2 walking policy as a signal block: wraps the exported ONNX actor and owns
/// everything the raw network needs around it, replicating the training-side conventions from
/// Open_Duck_Playground's <c>mujoco_infer.py</c>:
///
///   obs[101] = gyro(3) + accelerometer(3) + commands(7) + (joint_pos − home)(14)
///            + 0.05·joint_vel(14) + last/last²/last³ action(42) + motor_targets(14)
///            + foot_contacts(2) + gait_phase(2)
///
/// with actions mapped to motor targets as <c>home + action·0.25</c>, rate-limited to the servo's
/// 5.24 rad/s, at a fixed 50 Hz policy rate (the block self-paces against sim time and holds its
/// last targets between policy ticks). Joint vectors cross the wires in ovphysx DOF order and are
/// remapped to/from the MuJoCo actuator order the policy was trained with.
/// </summary>
public sealed class DuckPolicyBlock : ISignalBlock
{
    // Training-side conventions (Open_Duck_Playground open_duck_mini_v2).
    private const double PolicyDt = 0.02;        // sim_dt 0.002 × decimation 10
    private const double ActionScale = 0.25;
    private const double DofVelScale = 0.05;
    private const double MaxMotorVelocity = 5.24;    // rad/s (sts3215)
    private const int GaitPeriodSteps = 27;          // PolyReferenceMotion.nb_steps_in_period
    private const double AccelXOffset = 1.3;         // bias applied in mujoco_infer.get_obs
    private const double ContactHeight = 0.045;      // foot-origin z when planted is ~0.034-0.042

    /// <summary>MuJoCo actuator order (the policy's joint convention).</summary>
    private static readonly string[] MujocoOrder =
    [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ];

    /// <summary>The home pose ("home" keyframe) in MuJoCo actuator order, radians.</summary>
    private static readonly double[] Home =
    [
        0.002, 0.053, -0.63, 1.368, -0.784,
        0, 0, 0, 0,
        -0.003, -0.065, 0.635, 1.379, -0.796,
    ];

    /// <summary>ovphysx DOF order for the imported duck articulation (PhysX breadth-first).</summary>
    private static readonly string[] DefaultPhysxOrder =
    [
        "left_hip_yaw", "right_hip_yaw", "neck_pitch", "left_hip_roll", "right_hip_roll",
        "head_pitch", "left_hip_pitch", "right_hip_pitch", "head_yaw", "left_knee",
        "right_knee", "head_roll", "left_ankle", "right_ankle", "left_antenna", "right_antenna",
    ];

    private readonly OnnxPolicyBlock _policy;
    private readonly string[] _physxOrder;
    private readonly int[] _mjToPhysx; // MuJoCo actuator index -> physx DOF index

    // Policy state carried across ticks.
    private readonly double[] _lastAction = new double[14];
    private readonly double[] _lastLastAction = new double[14];
    private readonly double[] _lastLastLastAction = new double[14];
    private readonly double[] _motorTargets = (double[])Home.Clone();      // MuJoCo order, radians
    private readonly double[] _prevMotorTargets = (double[])Home.Clone();
    private double[] _outTargets;                                          // physx order, radians
    private readonly double[] _prevWorldLinVel = new double[3];
    private bool _hasPrevVel;
    private double _accumulator;
    private double _gaitStep;

    public string DisplayName => "Duck walk policy";
    public IReadOnlyList<BlockPin> InputPins { get; }
    public IReadOnlyList<BlockPin> OutputPins { get; }

    public DuckPolicyBlock(string onnxPath, string[]? physxDofOrder = null)
    {
        _policy = new OnnxPolicyBlock(onnxPath);
        _physxOrder = physxDofOrder ?? DefaultPhysxOrder;
        _mjToPhysx = MujocoOrder.Select(n => Array.IndexOf(_physxOrder, n)).ToArray();
        if (_mjToPhysx.Any(i => i < 0))
            throw new FmiException("Duck policy: the physx DOF order is missing an actuated joint name.");
        _outTargets = MapToPhysx(Home);

        InputPins =
        [
            new BlockPin("dof_pos", _physxOrder.Length, _physxOrder),
            new BlockPin("dof_vel", _physxOrder.Length, _physxOrder),
            new BlockPin("root_pose", 7, ["px", "py", "pz", "qx", "qy", "qz", "qw"]),
            new BlockPin("root_vel", 6, ["vx", "vy", "vz", "wx", "wy", "wz"]),
            new BlockPin("left_foot_pose", 7),
            new BlockPin("right_foot_pose", 7),
            new BlockPin("cmd_vx"), new BlockPin("cmd_vy"), new BlockPin("cmd_wz"),
            new BlockPin("cmd_neck_pitch"), new BlockPin("cmd_head_pitch"),
            new BlockPin("cmd_head_yaw"), new BlockPin("cmd_head_roll"),
        ];
        OutputPins =
        [
            new BlockPin("dof_targets", _physxOrder.Length, _physxOrder),
            new BlockPin("action", 14, MujocoOrder),
            new BlockPin("phase", 2, ["cos", "sin"]),
        ];
    }

    public void Start(double time, IReadOnlyDictionary<string, double> startValues) =>
        _policy.Start(time, startValues);

    public IReadOnlyDictionary<string, double[]> Step(
        IReadOnlyDictionary<string, double[]> inputs, double time, double dt)
    {
        // Self-pace to the 50 Hz training rate; between ticks the servos hold the last targets.
        _accumulator += dt;
        if (_accumulator >= PolicyDt)
        {
            _accumulator = Math.Min(_accumulator - PolicyDt, PolicyDt); // never spiral after a hitch
            PolicyTick(inputs);
        }

        return new Dictionary<string, double[]>
        {
            ["dof_targets"] = _outTargets,
            ["action"] = (double[])_lastAction.Clone(),
            ["phase"] = Phase(),
        };
    }

    private void PolicyTick(IReadOnlyDictionary<string, double[]> inputs)
    {
        double[] dofPos = Get(inputs, "dof_pos", _physxOrder.Length);
        double[] dofVel = Get(inputs, "dof_vel", _physxOrder.Length);
        double[] rootPose = Get(inputs, "root_pose", 7);
        double[] rootVel = Get(inputs, "root_vel", 6);

        // IMU: MuJoCo's gyro/accelerometer sensors are body-frame; physics reports world-frame.
        (double qx, double qy, double qz, double qw) = (rootPose[3], rootPose[4], rootPose[5], rootPose[6]);
        double[] gyro = RotateByInverse(qx, qy, qz, qw, rootVel[3], rootVel[4], rootVel[5]);

        double[] worldAccel = new double[3];
        if (_hasPrevVel)
            for (int i = 0; i < 3; i++) worldAccel[i] = (rootVel[i] - _prevWorldLinVel[i]) / PolicyDt;
        for (int i = 0; i < 3; i++) _prevWorldLinVel[i] = rootVel[i];
        _hasPrevVel = true;
        // Specific force: R⁻¹(a − g) with g = (0,0,−9.81); reads +9.81 up at rest, like the sensor.
        double[] accel = RotateByInverse(qx, qy, qz, qw, worldAccel[0], worldAccel[1], worldAccel[2] + 9.81);
        accel[0] += AccelXOffset;

        double[] commands =
        [
            Get1(inputs, "cmd_vx"), Get1(inputs, "cmd_vy"), Get1(inputs, "cmd_wz"),
            Get1(inputs, "cmd_neck_pitch"), Get1(inputs, "cmd_head_pitch"),
            Get1(inputs, "cmd_head_yaw"), Get1(inputs, "cmd_head_roll"),
        ];

        double leftContact = Get(inputs, "left_foot_pose", 7)[2] < ContactHeight ? 1 : 0;
        double rightContact = Get(inputs, "right_foot_pose", 7)[2] < ContactHeight ? 1 : 0;

        _gaitStep = (_gaitStep + 1) % GaitPeriodSteps;
        double[] phase = Phase();

        // Observation, exactly in training order (101 elements).
        var obs = new double[101];
        int o = 0;
        void Put(params double[] v) { foreach (double x in v) obs[o++] = x; }
        Put(gyro); Put(accel); Put(commands);
        for (int mj = 0; mj < 14; mj++) obs[o++] = dofPos[_mjToPhysx[mj]] - Home[mj];
        for (int mj = 0; mj < 14; mj++) obs[o++] = dofVel[_mjToPhysx[mj]] * DofVelScale;
        Put(_lastAction); Put(_lastLastAction); Put(_lastLastLastAction);
        Put(_motorTargets);
        Put(leftContact, rightContact);
        Put(phase);

        IReadOnlyDictionary<string, double[]> result = _policy.Step(
            new Dictionary<string, double[]> { ["obs"] = obs }, 0, PolicyDt);
        if (result.GetValueOrDefault("continuous_actions") is not { Length: 14 } action) return;

        Array.Copy(_lastLastAction, _lastLastLastAction, 14);
        Array.Copy(_lastAction, _lastLastAction, 14);
        Array.Copy(action, _lastAction, 14);

        // Motor model: targets around home, rate-limited to the servo's velocity.
        double maxDelta = MaxMotorVelocity * PolicyDt;
        for (int mj = 0; mj < 14; mj++)
        {
            double target = Home[mj] + action[mj] * ActionScale;
            _motorTargets[mj] = Math.Clamp(target, _prevMotorTargets[mj] - maxDelta, _prevMotorTargets[mj] + maxDelta);
            _prevMotorTargets[mj] = _motorTargets[mj];
        }
        _outTargets = MapToPhysx(_motorTargets);
    }

    private double[] Phase() =>
        [Math.Cos(_gaitStep / (double)GaitPeriodSteps * 2 * Math.PI),
         Math.Sin(_gaitStep / (double)GaitPeriodSteps * 2 * Math.PI)];

    // MuJoCo-order joint values into a physx-order DOF vector (unactuated DOFs stay zero/home-less).
    private double[] MapToPhysx(IReadOnlyList<double> mujocoValues)
    {
        var v = new double[_physxOrder.Length];
        for (int mj = 0; mj < 14; mj++) v[_mjToPhysx[mj]] = mujocoValues[mj];
        return v;
    }

    // Rotate a world vector into the body frame (by the inverse of the body quaternion).
    private static double[] RotateByInverse(double qx, double qy, double qz, double qw, double x, double y, double z)
    {
        // q⁻¹ v q for unit q — i.e. conjugate rotation.
        (qx, qy, qz) = (-qx, -qy, -qz);
        double tx = 2 * (qy * z - qz * y);
        double ty = 2 * (qz * x - qx * z);
        double tz = 2 * (qx * y - qy * x);
        return
        [
            x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx,
        ];
    }

    private static double[] Get(IReadOnlyDictionary<string, double[]> inputs, string pin, int width)
    {
        double[]? v = inputs.GetValueOrDefault(pin);
        if (v is { } src && src.Length >= width) return src;
        var padded = new double[width];
        if (v is not null) Array.Copy(v, padded, v.Length);
        return padded;
    }

    private static double Get1(IReadOnlyDictionary<string, double[]> inputs, string pin) =>
        inputs.GetValueOrDefault(pin) is { Length: > 0 } v ? v[0] : 0;

    public void Dispose() => _policy.Dispose();
}
