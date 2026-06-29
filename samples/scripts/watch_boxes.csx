// ovGemelli behavior script — runs once per frame with `sim`, `frame`, `time` in scope.
// Reads the live rigid-body poses over IPC and reports how far the boxes have fallen.
// (Hot-reloaded: edit and save while the twin runs.)

if (frame % 20 == 0)
{
    // RigidBodyPose rows are [px,py,pz, qx,qy,qz,qw]; this scene is Z-up, so pz is height.
    float[] poses = sim.Read(RigidBodyPose, "/World/Cube*");
    int n = poses.Length / 7;
    if (n > 0)
    {
        float minZ = float.MaxValue, maxZ = float.MinValue;
        for (int i = 0; i < n; i++)
        {
            float z = poses[i * 7 + 2];
            minZ = Math.Min(minZ, z);
            maxZ = Math.Max(maxZ, z);
        }
        print($"{n} boxes — height z range [{minZ:F2} .. {maxZ:F2}]");
    }
}
