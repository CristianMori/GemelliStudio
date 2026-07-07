using pxr;
using UniversalSceneDescription;

namespace Gemelli.Fmi;

/// <summary>One variable binding between an FMI model and a USD attribute (ovfmi FmuMapping prim).</summary>
public sealed record FmiMapping(string FmuVariable, string UsdAttribute, bool IsInput, int Offset, int Count);

/// <summary>Mappings grouped by their USD target prim (ovfmi FmuConnection prim).</summary>
public sealed record FmiConnection(string TargetPath, IReadOnlyList<FmiMapping> Mappings);

/// <summary>What kind of behavior model an instance prim references.</summary>
public enum FmiInstanceKind { Fmu, Ssp, Onnx }

/// <summary>One FmuInstance/SspInstance/OnnxInstance prim: its archive and connections.
/// <paramref name="Preset"/> selects a specialized host wrapper (e.g. "open_duck_mini_v2" wraps
/// the raw ONNX actor in <see cref="DuckPolicyBlock"/>'s observation/action conventions).</summary>
public sealed record FmiInstanceConfig(
    string PrimPath, string ArchivePath, FmiInstanceKind Kind, IReadOnlyList<FmiConnection> Connections,
    string? Preset = null)
{
    public bool IsSsp => Kind == FmiInstanceKind.Ssp;
}

/// <summary>A presence-sensor volume: world-space sphere derived from the target prim's transform.</summary>
public sealed record OverlapSensor(string PrimPath, float X, float Y, float Z, float Radius);

/// <summary>Everything the FMI runtime needs from a scene, captured once at load.</summary>
public sealed record FmiSceneConfig(
    IReadOnlyList<FmiInstanceConfig> Instances,
    IReadOnlyDictionary<string, OverlapSensor> OverlapSensors,
    IReadOnlyDictionary<string, Dictionary<string, double[]>> InitialAttributeValues);

/// <summary>
/// Reads the ovfmi USD-FMI schema from a stage: <c>FmuInstance</c>/<c>SspInstance</c> prims with
/// <c>FmuConnection</c> → <c>FmuMapping</c> children (see docs/USD-FMI-SCHEMA.md in ovfmi). Also
/// captures the initial values of USD attributes used as FMU inputs, and the world-space sphere of
/// every <c>physx:overlap</c> sensor target — so the runtime never needs the stage again.
/// </summary>
public static class FmiSchema
{
    public const string PhysxPosition = "physx:position";
    public const string PhysxVelocity = "physx:velocity";
    public const string PhysxForce = "physx:force";
    public const string PhysxOverlap = "physx:overlap";
    public const string DriveTargetVelocity = "drive:angular:physics:targetVelocity";

    // Vector routings beyond the ovfmi schema: whole articulation DOF vectors (the endpoint's
    // TargetPath is the articulation pattern; element order/labels are the DOF names), and whole
    // rigid-body state vectors (pose = px py pz qx qy qz qw; velocity = linear xyz + angular xyz).
    public const string DofPositions = "fmi:dofPositions";           // source: measured positions
    public const string DofVelocities = "fmi:dofVelocities";         // source: measured velocities
    public const string DofPositionTargets = "fmi:dofPositionTargets";   // sink: drive targets
    public const string DofVelocityTargets = "fmi:dofVelocityTargets";   // sink: drive targets
    public const string BodyPose = "fmi:bodyPose";                   // source: rigid-body pose [7]
    public const string BodyVelocity = "fmi:bodyVelocity";           // source: rigid-body velocity [6]

    /// <summary>Attributes routed through physics rather than read/written as USD attributes.</summary>
    public static bool IsPhysicsAttribute(string attr) =>
        attr is PhysxPosition or PhysxVelocity or PhysxForce or PhysxOverlap or DriveTargetVelocity
            or DofPositions or DofVelocities or DofPositionTargets or DofVelocityTargets
            or BodyPose or BodyVelocity;

    /// <summary>Returns the scene's FMI configuration, or null if it contains no FMI prims.</summary>
    public static FmiSceneConfig? Load(string usdPath)
    {
        UsdRuntime.Initialize();
        using UsdStage stage = UsdStage.Open(usdPath);
        if (stage is null) throw new FmiException($"Could not open USD stage: {usdPath}");

        string layerDir = Path.GetDirectoryName(Path.GetFullPath(usdPath)) ?? ".";
        var instances = new List<FmiInstanceConfig>();
        var sensors = new Dictionary<string, OverlapSensor>();
        var initialValues = new Dictionary<string, Dictionary<string, double[]>>();
        using var xf = new UsdGeomXformCache();

        foreach (UsdPrim prim in stage.Traverse())
        {
            string type = prim.GetTypeName().GetString();
            FmiInstanceKind kind;
            string assetAttr;
            switch (type)
            {
                case "FmuInstance": kind = FmiInstanceKind.Fmu; assetAttr = "fmi:fmu"; break;
                case "SspInstance": kind = FmiInstanceKind.Ssp; assetAttr = "fmi:ssp"; break;
                case "OnnxInstance": kind = FmiInstanceKind.Onnx; assetAttr = "fmi:onnx"; break;
                default: continue;
            }
            if (!ReadBool(prim, "fmi:enabled", defaultValue: true)) continue;

            string? archive = ReadAssetPath(prim, assetAttr, layerDir);
            if (archive is null)
                throw new FmiException($"{prim.GetPath().GetString()} has no {assetAttr} asset.");

            var connections = new List<FmiConnection>();
            foreach (UsdPrim conn in prim.GetChildren())
            {
                if (conn.GetTypeName().GetString() != "FmuConnection") continue;
                if (!ReadBool(conn, "fmi:enabled", defaultValue: true)) continue;

                foreach (string target in ReadTargets(conn))
                {
                    var mappings = new List<FmiMapping>();
                    foreach (UsdPrim map in conn.GetChildren())
                    {
                        if (map.GetTypeName().GetString() != "FmuMapping") continue;
                        string? fmuVar = ReadToken(map, "fmi:fmuAttribute");
                        string? usdAttr = ReadToken(map, "fmi:usdAttribute");
                        string? direction = ReadToken(map, "fmi:direction");
                        (int offset, int count) = ReadInt2(map, "fmi:usdMapping");
                        if (fmuVar is null || usdAttr is null || direction is null) continue;
                        mappings.Add(new FmiMapping(fmuVar, usdAttr, direction == "input", offset, count));
                    }
                    if (mappings.Count == 0) continue;
                    connections.Add(new FmiConnection(target, mappings));

                    // Capture what the runtime will need for each mapping kind.
                    foreach (FmiMapping m in mappings)
                    {
                        if (m.UsdAttribute == PhysxOverlap && !sensors.ContainsKey(target))
                            sensors[target] = ReadSensor(stage, xf, target);
                        else if (m.IsInput && !IsPhysicsAttribute(m.UsdAttribute))
                            CaptureInitialValue(stage, initialValues, target, m.UsdAttribute);
                    }
                }
            }

            instances.Add(new FmiInstanceConfig(
                prim.GetPath().GetString(), archive, kind, connections, ReadToken(prim, "fmi:preset")));
        }

        return instances.Count == 0 ? null : new FmiSceneConfig(instances, sensors, initialValues);
    }

    // FmuConnection's `rel fmi:targets` — one or more target prims.
    private static List<string> ReadTargets(UsdPrim conn)
    {
        var result = new List<string>();
        UsdRelationship rel = conn.GetRelationship(new TfToken("fmi:targets"));
        if (!rel.IsValid()) return result;
        SdfPathVector targets = rel.GetTargets();
        for (int i = 0; i < targets.Count; i++) result.Add(targets[i].GetString());
        return result;
    }

    // The sensor's world position (composed transform) plus its sphere radius (a Sphere child's
    // `radius` attribute when present, else the ovfmi default of 0.1).
    private static OverlapSensor ReadSensor(UsdStage stage, UsdGeomXformCache xf, string targetPath)
    {
        UsdPrim prim = stage.GetPrimAtPath(new SdfPath(targetPath));
        if (!prim.IsValid())
            throw new FmiException($"physx:overlap sensor target does not exist in the stage: {targetPath}");

        GfMatrix4d world = xf.GetLocalToWorldTransform(prim);
        GfVec3d pos = world.ExtractTranslation();

        float radius = 0.1f;
        foreach (UsdPrim child in prim.GetChildren())
        {
            UsdAttribute r = child.GetAttribute(new TfToken("radius"));
            if (!r.IsValid()) continue;
            try { double d = r.Get(UsdTimeCode.Default()); radius = (float)d; break; } catch { /* keep default */ }
        }
        return new OverlapSensor(targetPath, (float)pos[0], (float)pos[1], (float)pos[2], radius);
    }

    private static void CaptureInitialValue(
        UsdStage stage, Dictionary<string, Dictionary<string, double[]>> into, string primPath, string attrName)
    {
        if (into.TryGetValue(primPath, out var attrs) && attrs.ContainsKey(attrName)) return;
        UsdPrim prim = stage.GetPrimAtPath(new SdfPath(primPath));
        if (!prim.IsValid()) return;
        double[]? components = ReadComponents(prim, attrName);
        if (components is null) return;
        if (!into.TryGetValue(primPath, out attrs)) into[primPath] = attrs = new Dictionary<string, double[]>();
        attrs[attrName] = components;
    }

    // Reads an attribute's value as flat doubles: vec3d/vec3f/double/float covered (the component
    // selector in the mapping indexes into this array).
    private static double[]? ReadComponents(UsdPrim prim, string attrName)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return null;
        try { GfVec3d v = a.Get(UsdTimeCode.Default()); return [v[0], v[1], v[2]]; } catch { }
        try { GfVec3f v = a.Get(UsdTimeCode.Default()); return [v[0], v[1], v[2]]; } catch { }
        try { double d = a.Get(UsdTimeCode.Default()); return [d]; } catch { }
        try { float f = a.Get(UsdTimeCode.Default()); return [f]; } catch { }
        return null;
    }

    private static bool ReadBool(UsdPrim prim, string attrName, bool defaultValue)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return defaultValue;
        try { bool b = a.Get(UsdTimeCode.Default()); return b; } catch { return defaultValue; }
    }

    private static string? ReadToken(UsdPrim prim, string attrName)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return null;
        try { TfToken t = a.Get(UsdTimeCode.Default()); return t.GetString(); } catch { }
        try { string s = a.Get(UsdTimeCode.Default()); return s; } catch { }
        return null;
    }

    private static (int Offset, int Count) ReadInt2(UsdPrim prim, string attrName)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return (0, 0);
        try { GfVec2i v = a.Get(UsdTimeCode.Default()); return (v[0], v[1]); } catch { return (0, 0); }
    }

    // The asset's resolved absolute path; falls back to layer-relative resolution of the authored path.
    private static string? ReadAssetPath(UsdPrim prim, string attrName, string layerDir)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return null;
        try
        {
            SdfAssetPath ap = a.Get(UsdTimeCode.Default());
            string resolved = ap.GetResolvedPath();
            if (!string.IsNullOrEmpty(resolved)) return resolved;
            string authored = ap.GetAssetPath();
            if (string.IsNullOrEmpty(authored)) return null;
            return Path.GetFullPath(Path.Combine(layerDir, authored));
        }
        catch { return null; }
    }
}
