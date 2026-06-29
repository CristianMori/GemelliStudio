using pxr;
using UniversalSceneDescription;

// Adds instance-segmentation synthetic-data output to an Isaac Sim–exported USD so ovrtx can render it:
//   1. semantic "class" labels on the top-level world objects (so the segmenter has things to label),
//   2. an InstanceSegmentationSD RenderVar added to the sensor product's orderedVars,
//   3. the two SyntheticData OmniGraph nodes (texture->buffer copy + buffer pointer) that the depth
//      var already uses, cloned for InstanceSegmentationSD.
// Usage: usd-addseg <inUsd> <outUsd> [productLeafName]
//   productLeafName: the RenderProduct to augment; defaults to the first non-viewport product found.

UsdRuntime.Initialize();

string inUsd = args.Length > 0 ? args[0] : @"C:\DataDrive\ovGemelli\scenes\franka_studio.usda";
string outUsd = args.Length > 1 ? args[1] : @"C:\DataDrive\ovGemelli\scenes\franka_studio_seg.usda";
string? productArg = args.Length > 2 ? args[2] : null;

const string VarName = "InstanceSegmentationSD";
const string RenderRoot = "/Render/OmniverseKit/HydraTextures";

using UsdStage stage = UsdStage.Open(inUsd);

// --- locate the sensor render product (the non-viewport one) ---
string? product = productArg;
foreach (UsdPrim p in stage.Traverse())
{
    if (p.GetTypeName().GetString() != "RenderProduct") continue;
    string name = p.GetName();
    if (name.Contains("ViewportTexture")) continue;
    if (product is null) product = name;
}
if (product is null) { Console.Error.WriteLine("No non-viewport RenderProduct found."); return 1; }
Console.WriteLine($"Augmenting render product '{product}'.");

// --- 1. semantic class labels on top-level world objects ---
int labeled = 0;
UsdPrim world = stage.GetPrimAtPath(new SdfPath("/World"));
if (world.IsValid())
{
    foreach (UsdPrim child in world.GetChildren())
    {
        string t = child.GetTypeName().GetString();
        if (t is "PhysicsScene" or "Camera" or "RenderSettings" || t.EndsWith("Light")) continue;
        ApplySemanticLabel(child, child.GetName());
        labeled++;
    }
}
Console.WriteLine($"Applied semantic 'class' labels to {labeled} world objects.");

// --- 2. RenderVar prim + add to the product's orderedVars ---
UsdPrim var = stage.DefinePrim(new SdfPath($"/Render/Vars/{VarName}"), new TfToken());
Attr(var, "sourceName", SdfValueTypeNames.String, custom: true).Set(VarName);

UsdPrim productPrim = stage.GetPrimAtPath(new SdfPath($"{RenderRoot}/{product}"));
UsdRelationship ordered = productPrim.GetRelationship(new TfToken("orderedVars"));
if (!ordered.IsValid()) ordered = productPrim.CreateRelationship(new TfToken("orderedVars"), false);
ordered.AddTarget(new SdfPath($"/Render/Vars/{VarName}"));
Console.WriteLine($"Added /Render/Vars/{VarName} to orderedVars.");

// --- 3. clone the two SyntheticData graph nodes the depth var uses ---
string postRenderPipe = $"{RenderRoot}/{product}/PostRender/SDGPipeline";
string gpuEntry = $"{postRenderPipe}/{product}_GpuInteropEntry";
UsdPrim copyNode = stage.DefinePrim(new SdfPath($"{postRenderPipe}/{product}_{VarName}PostCopyToBuff"), new TfToken("OmniGraphNode"));
Attr(copyNode, "inputs:cudaCopyNoStride", SdfValueTypeNames.Bool, true).Set(true);
Connect(Attr(copyNode, "inputs:exec", SdfValueTypeNames.UInt, true), $"{gpuEntry}.outputs:exec");
Connect(Attr(copyNode, "inputs:gpu", SdfValueTypeNames.UInt64, true), $"{gpuEntry}.outputs:gpu");
Attr(copyNode, "inputs:renderVar", SdfValueTypeNames.Token, true).Set(new TfToken(VarName));
Attr(copyNode, "inputs:renderVarBufferSuffix", SdfValueTypeNames.String, true).Set("buff");
Connect(Attr(copyNode, "inputs:rp", SdfValueTypeNames.UInt64, true), $"{gpuEntry}.outputs:rp");
Attr(copyNode, "node:type", SdfValueTypeNames.Token, false).Set(new TfToken("omni.syntheticdata.SdPostRenderVarTextureToBuffer"));
Attr(copyNode, "node:typeVersion", SdfValueTypeNames.Int, false).Set(1);
Attr(copyNode, "outputs:exec", SdfValueTypeNames.UInt, true);
Attr(copyNode, "outputs:renderVar", SdfValueTypeNames.Token, true);

string postProcPipe = "/Render/PostProcess/SDGPipeline";
string dispatch = $"{postProcPipe}/{product}_PostProcessDispatch";
UsdPrim ptrNode = stage.DefinePrim(new SdfPath($"{postProcPipe}/{product}_{VarName}buffPtr"), new TfToken("OmniGraphNode"));
Connect(Attr(ptrNode, "inputs:exec", SdfValueTypeNames.UInt, true), $"{dispatch}.outputs:exec");
Connect(Attr(ptrNode, "inputs:renderResults", SdfValueTypeNames.UInt64, true), $"{dispatch}.outputs:renderResults");
Attr(ptrNode, "inputs:renderVar", SdfValueTypeNames.Token, true).Set(new TfToken($"{VarName}buff"));
Attr(ptrNode, "node:type", SdfValueTypeNames.Token, false).Set(new TfToken("omni.syntheticdata.SdRenderVarPtr"));
Attr(ptrNode, "node:typeVersion", SdfValueTypeNames.Int, false).Set(2);
Attr(ptrNode, "outputs:__device", SdfValueTypeNames.Token, true).Set(new TfToken("cpu"));
Console.WriteLine("Cloned SyntheticData copy + pointer nodes for segmentation.");

Directory.CreateDirectory(Path.GetDirectoryName(outUsd)!);
stage.GetRootLayer().Export(outUsd);
Console.WriteLine($"Wrote {outUsd}.");
return 0;

// ---- helpers ----
// Create a (varying) attribute on the prim and return it for fluent .Set()/.Connect() chaining.
static UsdAttribute Attr(UsdPrim prim, string name, SdfValueTypeName type, bool custom) =>
    prim.CreateAttribute(new TfToken(name), type, custom, SdfVariability.SdfVariabilityVarying);

// Wire an attribute's input to another property's output (OmniGraph node connection).
static void Connect(UsdAttribute attr, string targetProperty) =>
    attr.AddConnection(new SdfPath(targetProperty));

static void ApplySemanticLabel(UsdPrim prim, string label)
{
    // Classic Kit/Replicator semantics schema (instance name "Semantics").
    prim.AddAppliedSchema(new TfToken("SemanticsAPI:Semantics"));
    Attr(prim, "semantic:Semantics:params:semanticType", SdfValueTypeNames.Token, true).Set(new TfToken("class"));
    Attr(prim, "semantic:Semantics:params:semanticData", SdfValueTypeNames.Token, true).Set(new TfToken(label));
}
