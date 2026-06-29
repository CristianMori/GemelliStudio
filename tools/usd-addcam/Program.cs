using System.Numerics;
using pxr;
using UniversalSceneDescription;

// Adds a perspective camera at /OmniverseKit_Persp (the camera the falling-boxes render product
// references but the file never defines) and exports a renderable copy. Z-up, meters.
// Usage: usd-addcam <inUsd> <outUsd> [camPath] [WxH]
//   WxH (optional): override every RenderProduct's resolution (e.g. 960x540) to trade quality for speed.

UsdRuntime.Initialize();

string inUsd = args.Length > 0 ? args[0] : @"C:\DataDrive\ovGemelli\native\ovphysx\ovphysx\samples\data\boxes_falling_on_groundplane.usda";
string outUsd = args.Length > 1 ? args[1] : @"C:\DataDrive\ovGemelli\out\boxes_with_camera.usda";
string camPath = args.Length > 2 ? args[2] : "/OmniverseKit_Persp";

(int W, int H)? res = null;
if (args.Length > 3 && args[3].Split('x') is [var ws, var hs] && int.TryParse(ws, out int rw) && int.TryParse(hs, out int rh))
    res = (rw, rh);

// Optional render mode token (e.g. "RealTimePathTracing" = fast, "PathTracing" = reference quality).
string? renderMode = args.Length > 4 && !string.IsNullOrWhiteSpace(args[4]) ? args[4] : null;

// Look-at framing (Z-up, meters). The 11 cubes form a row centered near (10, -1, 10) that falls
// onto the ground; frame the whole row from back and above.
var eye = new Vector3(10f, -30f, 13f);
var target = new Vector3(10f, -1f, 4f);
var worldUp = new Vector3(0f, 0f, 1f);

// USD camera looks down local -Z; +Y is up, +X is right. Build the camera-to-world basis.
Vector3 zAxis = Vector3.Normalize(eye - target);          // camera +Z = backward
Vector3 xAxis = Vector3.Normalize(Vector3.Cross(worldUp, zAxis));
Vector3 yAxis = Vector3.Cross(zAxis, xAxis);

// Row-vector (USD) 4x4: rows are the basis axes, last row is translation.
double[] m =
[
    xAxis.X, xAxis.Y, xAxis.Z, 0,
    yAxis.X, yAxis.Y, yAxis.Z, 0,
    zAxis.X, zAxis.Y, zAxis.Z, 0,
    eye.X,   eye.Y,   eye.Z,   1,
];

using UsdStage stage = UsdStage.Open(inUsd);

UsdGeomCamera cam = UsdGeomCamera.Define(stage, new SdfPath(camPath));
UsdPrim prim = cam.GetPrim();

var xformable = new UsdGeomXformable(prim);
UsdGeomXformOp op = xformable.AddTransformOp(UsdGeomXformOp.Precision.PrecisionDouble, new TfToken(), false);
op.Set(new GfMatrix4d(
    m[0], m[1], m[2], m[3],
    m[4], m[5], m[6], m[7],
    m[8], m[9], m[10], m[11],
    m[12], m[13], m[14], m[15]), UsdTimeCode.Default());

// Sensible perspective + clipping for the scene scale (cm-based focal/aperture USD convention).
cam.CreateFocalLengthAttr().Set(24.0f);
cam.CreateHorizontalApertureAttr().Set(36.0f);
cam.CreateVerticalApertureAttr().Set(20.25f);
cam.CreateClippingRangeAttr().Set(new GfVec2f(0.01f, 1000f));

// Add a dome (environment) light so surfaces are lit and the background isn't pure black.
// The sample's render product sets background source = domeLight but the file defines no dome.
UsdLuxDomeLight dome = UsdLuxDomeLight.Define(stage, new SdfPath("/Environment/DomeLight"));
dome.CreateIntensityAttr().Set(1000.0f);
Console.WriteLine("Added /Environment/DomeLight (intensity 1000).");

// Optionally override render resolution and/or render mode on every render product.
if (res is not null || renderMode is not null)
{
    int patched = 0;
    foreach (UsdPrim p in stage.Traverse())
    {
        if (p.GetTypeName().GetString() != "RenderProduct") continue;
        if (res is { } r)
        {
            UsdAttribute attr = p.GetAttribute(new TfToken("resolution"));
            if (!attr.IsValid()) attr = p.CreateAttribute(new TfToken("resolution"), SdfValueTypeNames.Int2, true, SdfVariability.SdfVariabilityUniform);
            attr.Set(new GfVec2i(r.W, r.H));
        }
        if (renderMode is not null)
        {
            UsdAttribute attr = p.GetAttribute(new TfToken("omni:rtx:rendermode"));
            if (!attr.IsValid()) attr = p.CreateAttribute(new TfToken("omni:rtx:rendermode"), SdfValueTypeNames.Token, true, SdfVariability.SdfVariabilityUniform);
            attr.Set(new TfToken(renderMode));
        }
        patched++;
    }
    string what = (res is { } rr ? $"resolution {rr.W}x{rr.H}" : "") + (res is not null && renderMode is not null ? ", " : "") + (renderMode is not null ? $"rendermode '{renderMode}'" : "");
    Console.WriteLine($"Set {what} on {patched} render product(s).");
}

Directory.CreateDirectory(Path.GetDirectoryName(outUsd)!);
stage.GetRootLayer().Export(outUsd);

Console.WriteLine($"Wrote {outUsd} with camera at {camPath} (eye {eye}, target {target}).");
