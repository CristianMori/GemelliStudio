using Nvidia.OvPhysx;

// Physics-only probe: load an Isaac Sim USD with ovphysx, find articulations, and dump what the
// ArticulationLinkPose binding exposes — to learn whether BodyNames are full prim paths or names.
// Usage: artic-probe <usd> [pattern]

string usd = args.Length > 0 ? args[0] : @"C:\DataDrive\IsaacSimSharp\out\urdf_import.usda";
string pattern = args.Length > 1 ? args[1] : "/World/*";
var device = args.Length > 2 && args[2] == "gpu" ? DeviceType.Gpu : DeviceType.Cpu;
Console.WriteLine($"device={device}");

using var physx = new PhysX(device: device);
var (_, load) = physx.AddUsd(usd);
load.Wait(TimeSpan.FromSeconds(30));
Console.WriteLine($"loaded {usd}");

// Inspect the articulation for IK: metadata, link names (with indices), and the Jacobian shape.
string artPattern = "/World/robot";
using (TensorBinding links = physx.CreateTensorBinding(TensorType.ArticulationLinkPose, artPattern, raiseIfEmpty: false))
{
    Console.WriteLine($"\n>>> Articulation '{artPattern}'  N={links.Count}");
    if (links.Count > 0)
    {
        Console.WriteLine($"    DofCount={links.DofCount}  BodyCount={links.BodyCount}  IsFixedBase={links.IsFixedBase}");
        Console.WriteLine($"    LinkPose shape=[{string.Join(",", links.Shape)}]");
        var names = links.BodyNames;
        for (int i = 0; i < names.Count; i++) Console.WriteLine($"      link[{i}] = {names[i]}");
        Console.WriteLine($"    DofNames: {string.Join(", ", links.DofNames)}");
    }
}
try
{
    using TensorBinding jac = physx.CreateTensorBinding(TensorType.ArticulationJacobian, artPattern, raiseIfEmpty: false);
    using TensorBinding dofTgt = physx.CreateTensorBinding(TensorType.ArticulationDofPositionTarget, artPattern, raiseIfEmpty: false);
    Console.WriteLine($"    Jacobian shape=[{string.Join(",", jac.Shape)}]  (cols={jac.Shape[^1]})");
    int cols = (int)jac.Shape[^1];
    // Drive the arm to a non-zero config and step a bit so the Jacobian is at a meaningful pose.
    var tgt = new float[(int)dofTgt.ElementCount];
    if (tgt.Length > 0) { tgt[0] = 0.4f; if (tgt.Length > 3) tgt[3] = -1.5f; if (tgt.Length > 5) tgt[5] = 1.5f; }
    dofTgt.Write(tgt);
    physx.StepNSync(40, 1f / 60f, 0f);
    float[] j = jac.Read();
    // TCP = panda_hand = link index 8; fixed base removes link0, so Jacobian block = 8-1 = 7; linear rows = 7*6 .. +2.
    int tcpRow = 7 * 6;
    Console.WriteLine($"    TCP(panda_hand) linear-Jacobian rows {tcpRow}..{tcpRow + 2}:");
    for (int r = 0; r < 3; r++)
        Console.WriteLine($"      d{"xyz"[r]}/dq = [{string.Join(", ", Enumerable.Range(0, cols).Select(c => j[(tcpRow + r) * cols + c].ToString("F3")))}]");
}
catch (Exception ex) { Console.WriteLine($"    Jacobian ERROR: {ex.Message.Split('\n')[0]}"); }
