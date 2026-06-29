// Drive the Franka TCP (panda_hand = link index 8) around a horizontal circle using differential IK.
// MoveTcp does one damped-least-squares IK step per call toward a world-space target.
float t = (float)time;
float cx = 0.40f, cy = 0.0f, cz = 0.50f, r = 0.15f;   // circle center + radius (meters, Z-up)
float tx = cx + r * MathF.Cos(t);
float ty = cy + r * MathF.Sin(t);

var tcp = MoveTcp("/World/robot", 8, tx, ty, cz);

if (frame % 20 == 0 && tcp.HasValue)
    print($"target=({tx:F2},{ty:F2},{cz:F2})  tcp=({tcp.Value.X:F2},{tcp.Value.Y:F2},{tcp.Value.Z:F2})");
