using pxr;
using UniversalSceneDescription;

// USD authoring spike: prove load → inspect → edit → SAVE-BACK → reload round-trips with USD.NET.
// This validates the UsdDocument backing for ovGemelli's editing + save-back milestone.

UsdRuntime.Initialize();

string sample = args.Length > 0
    ? args[0]
    : @"C:\DataDrive\ovGemelli\native\ovphysx\ovphysx\samples\data\boxes_falling_on_groundplane.usda";

Console.WriteLine("=== 1. Open an existing (Omniverse-authored) USD and inspect it ===");
using (UsdStage stage = UsdStage.Open(sample))
{
    int count = 0;
    foreach (UsdPrim prim in stage.Traverse())
    {
        if (count++ < 15)
            Console.WriteLine($"  {prim.GetPath().GetString(),-55} <{prim.GetTypeName().GetString()}>");
    }
    Console.WriteLine($"  ... {count} prims total");

    UsdPrim cube = stage.GetPrimAtPath(new SdfPath("/World/Cube1"));
    Console.WriteLine($"  /World/Cube1 valid: {cube.IsValid()}");
}

Console.WriteLine();
Console.WriteLine("=== 2. Author a new stage, set a transform, SAVE, reload, verify ===");
string outPath = Path.Combine(Path.GetTempPath(), "ovgemelli_usd_spike.usda");

using (UsdStage stage = UsdStage.CreateNew(outPath))
{
    UsdGeomXform.Define(stage, new SdfPath("/World"));
    UsdPrim box = UsdGeomCube.Define(stage, new SdfPath("/World/Box")).GetPrim();

    var xform = new UsdGeomXformCommonAPI(box);
    xform.SetTranslate(new GfVec3d(1.0, 2.0, 3.0));

    stage.Save();
    Console.WriteLine($"  saved -> {outPath}");
}

using (UsdStage reloaded = UsdStage.Open(outPath))
{
    UsdPrim box = reloaded.GetPrimAtPath(new SdfPath("/World/Box"));
    var xform = new UsdGeomXformCommonAPI(box);

    xform.GetXformVectors(out GfVec3d translate, out GfVec3f rotate, out GfVec3f scale,
        out GfVec3f pivot, out UsdGeomXformCommonAPI.RotationOrder order, new UsdTimeCode(0));
    _ = (rotate, scale, pivot, order);

    Console.WriteLine($"  reloaded /World/Box translate = ({translate[0]}, {translate[1]}, {translate[2]})");
    Console.WriteLine(translate[0] == 1.0 && translate[1] == 2.0 && translate[2] == 3.0
        ? "  ROUND-TRIP OK ✔"
        : "  ROUND-TRIP MISMATCH �’");
}

Console.WriteLine();
Console.WriteLine("=== 3. Show the saved .usda text (proves human-readable save-back) ===");
Console.WriteLine(File.ReadAllText(outPath));
