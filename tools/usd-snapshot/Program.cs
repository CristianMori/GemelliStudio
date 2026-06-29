using pxr;
using UniversalSceneDescription;

// Save-back: bake the twin's current rigid-body world poses (and, optionally, articulation joint state)
// into a standalone USD, so the edited/simulated state becomes the scene's new initial conditions
// (reload → physics starts from here).
// Usage: usd-snapshot <inUsd> <outUsd> <posesFile> [jointsFile]
//   posesFile:  one body per line  ->  <primPath> <px> <py> <pz> <qx> <qy> <qz> <qw>
//               (position + xyzw quaternion, exactly the ovphysx RigidBodyPose row layout)
//   jointsFile: one DOF per line   ->  <jointName> <value>   (radians for revolute, metres for prismatic)

if (args.Length < 3) { Console.Error.WriteLine("Usage: usd-snapshot <inUsd> <outUsd> <posesFile> [jointsFile]"); return 1; }
string inUsd = args[0], outUsd = args[1], posesFile = args[2];
string? jointsFile = args.Length > 3 ? args[3] : null;

UsdRuntime.Initialize();
using UsdStage stage = UsdStage.Open(inUsd);

int written = 0, skipped = 0;
foreach (string line in File.ReadAllLines(posesFile))
{
    string s = line.Trim();
    if (s.Length == 0 || s.StartsWith('#')) continue;
    string[] f = s.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
    if (f.Length < 8) { skipped++; continue; }

    string path = f[0];
    float px = P(f[1]), py = P(f[2]), pz = P(f[3]);
    float qx = P(f[4]), qy = P(f[5]), qz = P(f[6]), qw = P(f[7]);

    UsdPrim prim = stage.GetPrimAtPath(new SdfPath(path));
    if (!prim.IsValid()) { skipped++; continue; }

    var xf = new UsdGeomXformable(prim);
    xf.ClearXformOpOrder();           // drop existing ops…
    xf.SetResetXformStack(true);      // …and ignore parent transforms: our matrix is world-space
    UsdGeomXformOp op = xf.AddTransformOp(UsdGeomXformOp.Precision.PrecisionDouble, new TfToken(), false);
    op.Set(WorldMatrix(px, py, pz, qx, qy, qz, qw), UsdTimeCode.Default());
    written++;
}

// Optional: bake articulation joint state so a posed robot reloads in the same configuration.
int joints = 0;
if (jointsFile is not null && File.Exists(jointsFile))
{
    var values = new Dictionary<string, float>();
    foreach (string line in File.ReadAllLines(jointsFile))
    {
        string[] f = line.Trim().Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
        if (f.Length >= 2) values[f[0]] = P(f[1]);
    }
    foreach (UsdPrim prim in stage.Traverse())
    {
        string t = prim.GetTypeName().GetString();
        bool angular = t == "PhysicsRevoluteJoint";
        bool linear = t == "PhysicsPrismaticJoint";
        if (!angular && !linear) continue;
        if (!values.TryGetValue(prim.GetName(), out float v)) continue;

        string axis = angular ? "angular" : "linear";
        // ovphysx DOF positions are radians for revolute joints, but UsdPhysics angular drive/state are in
        // DEGREES (linear/prismatic stay in metres). Convert so the reloaded pose matches the sim.
        float usdVal = angular ? v * (180f / MathF.PI) : v;
        SetFloat(prim, $"drive:{axis}:physics:targetPosition", usdVal); // joint drive holds the pose…
        SetFloat(prim, $"state:{axis}:physics:position", usdVal);       // …and it starts there
        joints++;
    }
}

stage.GetRootLayer().Export(outUsd);
Console.WriteLine($"Snapshot wrote {written} body transforms, {joints} joint states ({skipped} bodies skipped) -> {outUsd}");
return 0;

// Get-or-create a Float attribute on the prim and set it (used for joint drive/state authoring).
static void SetFloat(UsdPrim prim, string attrName, float value)
{
    UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
    if (!a.IsValid()) a = prim.CreateAttribute(new TfToken(attrName), SdfValueTypeNames.Float, false, SdfVariability.SdfVariabilityVarying);
    a.Set(value, UsdTimeCode.Default());
}

// Parse a float with invariant culture so '.' is the decimal separator regardless of locale.
static float P(string v) => float.Parse(v, System.Globalization.CultureInfo.InvariantCulture);

// USD uses row-vector convention (point * matrix); rows are the basis axes, last row the translation.
static GfMatrix4d WorldMatrix(float px, float py, float pz, float qx, float qy, float qz, float qw)
{
    // Normalize the quaternion, then build its rotation matrix (row-vector form).
    double n = Math.Sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (n < 1e-9) { qw = 1; qx = qy = qz = 0; n = 1; }
    double x = qx / n, y = qy / n, z = qz / n, w = qw / n;
    double xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;

    return new GfMatrix4d(
        1 - 2 * (yy + zz), 2 * (xy + wz),     2 * (xz - wy),     0,
        2 * (xy - wz),     1 - 2 * (xx + zz), 2 * (yz + wx),     0,
        2 * (xz + wy),     2 * (yz - wx),     1 - 2 * (xx + yy), 0,
        px,                py,                pz,                1);
}
