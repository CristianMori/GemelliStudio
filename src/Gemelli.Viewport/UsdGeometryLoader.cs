using System.Numerics;
using pxr;
using UniversalSceneDescription;

namespace Gemelli.Viewport;

/// <summary>
/// One renderable mesh: interleaved position+normal+uv (8 floats/vertex, 3 vertices/triangle), with smooth
/// per-vertex normals so curved surfaces shade smoothly. Geometry is pre-baked into either the controlling
/// rigid body's local frame (<see cref="BodyPath"/> non-null → per-frame model matrix is that body's live
/// pose) or world space (null → static, model = identity). <see cref="TexturePath"/> keys into
/// <see cref="GeometryResult.Textures"/>; the color acts as a tint when a texture is present.
/// </summary>
public sealed record RenderMesh(float[] Vertices, Vector3 Color, string? BodyPath, string? TexturePath = null, float Alpha = 1f);

/// <summary>A decoded albedo texture (RGBA8, bottom-up rows to match GL/USD texture-coordinate origin).</summary>
public sealed record TextureData(int Width, int Height, byte[] Rgba);

/// <summary>Loaded geometry plus the world-space bounds of the renderable (non-ground) objects at load.</summary>
public sealed record GeometryResult(
    List<RenderMesh> Meshes, Vector3 Center, float Radius, Dictionary<string, Matrix4x4> BodyInverse,
    Dictionary<string, TextureData> Textures);

/// <summary>
/// Extracts displayable geometry from an Isaac Sim USD via USD.NET: direct meshes (smooth-normalled), USD
/// shape prims (Cube/Sphere/Cylinder/Cone/Capsule/Plane), and instanced geometry (robot links reference
/// <c>__Prototype_*</c>). Per-prim color comes from <c>primvars:displayColor</c> or the bound material's
/// diffuse, else a stable hash tint. Each mesh follows its nearest ancestor rigid body. Read-only.
/// </summary>
public sealed class UsdGeometryLoader
{
    private readonly record struct Vtx(Vector3 Pos, Vector3 Nrm, Vector2 Uv = default);

    /// <summary>What a bound material contributes: a constant tint and/or an albedo texture path.</summary>
    private readonly record struct MaterialLook(Vector3? Tint, string? Texture);

    // Diffuse-color shader inputs we recognize: UsdPreviewSurface (diffuseColor) and Omniverse OmniPBR/MDL
    // (diffuse_color_constant). Checked in order; first valid GfVec3f wins.
    private static readonly string[] DiffuseAttrs = { "inputs:diffuseColor", "inputs:diffuse_color_constant" };

    private readonly HashSet<string> _bodies;
    private readonly Dictionary<string, Matrix4x4> _bodyInverse = new();
    private readonly UsdStage _stage;
    private readonly UsdGeomXformCache _xf = new();
    private readonly List<RenderMesh> _out = new();
    private readonly Dictionary<string, MaterialLook> _materialCache = new();  // material prim path -> look
    private readonly Dictionary<string, TextureData> _textures = new();        // resolved file -> pixels
    private Vector3 _wMin = new(float.MaxValue), _wMax = new(float.MinValue);

    private UsdGeometryLoader(UsdStage stage, IReadOnlyCollection<string> rigidBodyPaths)
    {
        _stage = stage;
        _bodies = new HashSet<string>(rigidBodyPaths);
    }

    /// <summary>
    /// Opens the stage and extracts all renderable geometry. Returns the meshes plus a framing bound
    /// (center + radius) over the non-ground objects, defaulting to a sensible box if nothing was found.
    /// </summary>
    public static GeometryResult Load(string usdPath, IReadOnlyCollection<string> rigidBodyPaths)
    {
        UsdRuntime.Initialize();
        using UsdStage stage = UsdStage.Open(usdPath);
        var loader = new UsdGeometryLoader(stage, rigidBodyPaths);
        loader.Run();
        bool any = loader._wMin.X <= loader._wMax.X;
        Vector3 center = any ? (loader._wMin + loader._wMax) * 0.5f : new Vector3(0, 0, 0.4f);
        float radius = any ? MathF.Max(0.3f, (loader._wMax - loader._wMin).Length() * 0.5f) : 1.5f;
        return new GeometryResult(loader._out, center, radius, loader._bodyInverse, loader._textures);
    }

    // Two passes' worth of work: first cache each rigid body's inverse load-time (rigid) transform for baking,
    // then traverse the whole stage emitting meshes, USD shape prims, and instanced prototype meshes.
    private void Run()
    {
        foreach (string b in _bodies)
        {
            UsdPrim bp = _stage.GetPrimAtPath(new SdfPath(b));
            // Invert the RIGID part of the body's load-time transform: the runtime physics pose is rigid
            // (rotation + translation, no scale), so baking must drop any scale.
            if (bp.IsValid() && Matrix4x4.Invert(RigidPart(ToM(_xf.GetLocalToWorldTransform(bp))), out Matrix4x4 inv))
                _bodyInverse[b] = inv;
        }

        // Subtrees culled by purpose/visibility: Traverse() is depth-first pre-order, so a skipped
        // prim's descendants all arrive later with its path as a prefix.
        var skips = new List<string>();
        bool Skipped(string pth)
        {
            foreach (string s in skips) if (pth.StartsWith(s, StringComparison.Ordinal)) return true;
            return false;
        }

        foreach (UsdPrim prim in _stage.Traverse())
        {
            string type = prim.GetTypeName().GetString();
            string path = prim.GetPath().GetString();
            if (Skipped(path)) continue;
            if (!IsRenderable(prim)) { skips.Add(path + "/"); continue; }

            if (type == "Mesh")
                EmitMesh(new UsdGeomMesh(prim), ToM(_xf.GetLocalToWorldTransform(prim)), path);
            else if (ShapeVerts(prim, type) is { } sv)
                Emit(sv, ToM(_xf.GetLocalToWorldTransform(prim)), path, LookFor(prim, path).Color, // shapes carry no UVs → tint only
                    alpha: ReadOpacity(prim));

            // Instanced prim: walk the shared prototype's meshes and place each by composing its prototype-local
            // transform with this instance's world transform (the prototype is authored at the origin).
            if (prim.IsInstance())
            {
                UsdPrim proto = prim.GetPrototype();
                if (!proto.IsValid()) continue;
                Matrix4x4 instWorld = ToM(_xf.GetLocalToWorldTransform(prim));
                string protoRoot = proto.GetPath().GetString();
                foreach (UsdPrim cp in proto.GetDescendants())
                    if (cp.GetTypeName().GetString() == "Mesh" && IsRenderable(cp))
                    {
                        // Prefer the instance-proxy prim over the raw prototype child: material
                        // bindings authored inside an instanced payload only resolve with instance
                        // context (on the prototype side USD culls them as out-of-scope), and the
                        // proxy's transform is already the full world transform.
                        UsdPrim proxy = _stage.GetPrimAtPath(new SdfPath(path + cp.GetPath().GetString()[protoRoot.Length..]));
                        if (proxy.IsValid())
                            EmitMesh(new UsdGeomMesh(proxy), ToM(_xf.GetLocalToWorldTransform(proxy)), path);
                        else
                            EmitMesh(new UsdGeomMesh(cp), ToM(_xf.GetLocalToWorldTransform(cp)) * instWorld, path);
                    }
            }
        }
    }

    // Read a mesh, split its faces by GeomSubset (each binds its own material), and emit one RenderMesh per
    // distinct material color. Faces not covered by any subset use the mesh's own binding/displayColor/hash.
    // Smooth per-vertex normals are computed across the whole mesh so curved surfaces shade continuously even
    // across subset boundaries.
    private void EmitMesh(UsdGeomMesh mesh, Matrix4x4 geoToWorld, string path)
    {
        UsdPrim prim = mesh.GetPrim();
        VtVec3fArray pts = mesh.GetPointsAttr().Get(UsdTimeCode.Default());
        VtIntArray counts = mesh.GetFaceVertexCountsAttr().Get(UsdTimeCode.Default());
        VtIntArray idx = mesh.GetFaceVertexIndicesAttr().Get(UsdTimeCode.Default());
        if (pts is null || counts is null || idx is null || pts.size() == 0) return;

        int np = (int)pts.size();
        var p = new Vector3[np];
        for (int i = 0; i < np; i++) { GfVec3f q = pts[i]; p[i] = new Vector3(q[0], q[1], q[2]); }

        int nf = (int)counts.size(), ni = (int)idx.size();
        var n = new Vector3[np];
        // Triangles carry both POINT indices (positions/normals) and CORNER indices (face-varying UVs).
        var faceTris = new List<(int A, int B, int C, int Ca, int Cb, int Cc)>[nf];
        int cursor = 0;
        for (int f = 0; f < nf; f++)
        {
            int c = counts[f];
            var tl = new List<(int, int, int, int, int, int)>();
            if (c >= 3 && cursor + c <= ni)
                for (int k = 1; k < c - 1; k++)
                {
                    int a = idx[cursor], b = idx[cursor + k], d = idx[cursor + k + 1];
                    Vector3 fn = Vector3.Cross(p[b] - p[a], p[d] - p[a]);
                    n[a] += fn; n[b] += fn; n[d] += fn;
                    tl.Add((a, b, d, cursor, cursor + k, cursor + k + 1));
                }
            faceTris[f] = tl;
            cursor += c;
        }
        for (int i = 0; i < np; i++)
            n[i] = n[i].LengthSquared() > 1e-12f ? Vector3.Normalize(n[i]) : Vector3.UnitZ;

        Func<int, int, Vector2> uv = ReadUvLookup(prim, np, ni);

        // Per-face look: start with the mesh's own, then override faces named by each GeomSubset
        // (each subset can bind its own material — different texture and/or tint).
        (Vector3 Color, string? Texture) defLook = LookFor(prim, path);
        var faceLook = new (Vector3 Color, string? Texture)[nf];
        for (int f = 0; f < nf; f++) faceLook[f] = defLook;
        foreach (UsdPrim sub in prim.GetChildren())
        {
            if (sub.GetTypeName().GetString() != "GeomSubset") continue;
            MaterialLook ml = MaterialLookFor(sub);
            (Vector3, string?) sl = ml.Texture is not null
                ? (ml.Tint is { } st && st.X + st.Y + st.Z > 0.05f ? st : Vector3.One, ml.Texture)
                : (ml.Tint ?? defLook.Color, null);
            UsdAttribute ia = sub.GetAttribute(new TfToken("indices"));
            if (!ia.IsValid()) continue;
            try
            {
                VtIntArray si = ia.Get(UsdTimeCode.Default());
                if (si is null) continue;
                for (int i = 0; i < (int)si.size(); i++) { int f = si[i]; if ((uint)f < (uint)nf) faceLook[f] = sl; }
            }
            catch { }
        }

        // Bucket triangles by (texture, quantized color), emit one mesh per bucket.
        var buckets = new Dictionary<(string, int), (Vector3 color, string? tex, List<Vtx> verts)>();
        for (int f = 0; f < nf; f++)
        {
            (Vector3 col, string? tex) = faceLook[f];
            var key = (tex ?? "", ((int)(col.X * 255) << 16) | ((int)(col.Y * 255) << 8) | (int)(col.Z * 255));
            if (!buckets.TryGetValue(key, out var bk)) buckets[key] = bk = (col, tex, new List<Vtx>());
            foreach (var (a, b, c, ca, cb, cc) in faceTris[f])
            {
                bk.verts.Add(new Vtx(p[a], n[a], uv(a, ca)));
                bk.verts.Add(new Vtx(p[b], n[b], uv(b, cb)));
                bk.verts.Add(new Vtx(p[c], n[c], uv(c, cc)));
            }
        }
        float meshAlpha = ReadOpacity(prim);
        foreach (var (_, bucket) in buckets)
            Emit(bucket.verts, geoToWorld, path, bucket.color, bucket.tex, meshAlpha);
    }

    // Authored primvars:displayOpacity (constant across the prim), 1 = opaque. Lets scenes mark
    // helper geometry (sensor volumes) translucent in the nav viewport. NOTE: displayOpacity is a
    // BUILT-IN Gprim attribute, so GetAttribute returns a valid handle even when nothing is
    // authored and Get yields an empty array — only a non-empty array counts (an unauthored value
    // must not read as 0 = fully transparent).
    private static float ReadOpacity(UsdPrim prim)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken("primvars:displayOpacity"));
        if (!a.IsValid()) return 1f;
        try
        {
            VtFloatArray arr = a.Get(UsdTimeCode.Default());
            if (arr is not null && arr.size() > 0) return Math.Clamp(arr[0], 0f, 1f);
        }
        catch { }
        return 1f;
    }

    // Builds a (pointIndex, cornerIndex) → UV lookup from primvars:st. USD authors texcoords either
    // per point ("vertex" interpolation: one UV per point) or per face corner ("faceVarying": one UV
    // per faceVertexIndices entry, optionally indirected through primvars:st:indices). The array
    // length disambiguates. Returns a zero lookup when the mesh has no usable texcoords.
    private static Func<int, int, Vector2> ReadUvLookup(UsdPrim prim, int pointCount, int cornerCount)
    {
        static Func<int, int, Vector2> None() => (_, _) => default;
        try
        {
            UsdAttribute st = prim.GetAttribute(new TfToken("primvars:st"));
            if (!st.IsValid()) st = prim.GetAttribute(new TfToken("primvars:UVMap"));
            if (!st.IsValid()) return None();
            VtVec2fArray raw = st.Get(UsdTimeCode.Default());
            if (raw is null || raw.size() == 0) return None();
            var uvs = new Vector2[(int)raw.size()];
            for (int i = 0; i < uvs.Length; i++) { GfVec2f v = raw[i]; uvs[i] = new Vector2(v[0], v[1]); }

            int[]? indices = null;
            UsdAttribute ia = prim.GetAttribute(new TfToken("primvars:st:indices"));
            if (ia.IsValid())
                try
                {
                    VtIntArray si = ia.Get(UsdTimeCode.Default());
                    if (si is not null && si.size() > 0)
                    {
                        indices = new int[(int)si.size()];
                        for (int i = 0; i < indices.Length; i++) indices[i] = si[i];
                    }
                }
                catch { }

            if (indices is not null && indices.Length == cornerCount)
                return (_, corner) => (uint)corner < (uint)indices.Length && (uint)indices[corner] < (uint)uvs.Length
                    ? uvs[indices[corner]] : default;
            if (uvs.Length == cornerCount)
                return (_, corner) => (uint)corner < (uint)uvs.Length ? uvs[corner] : default;
            if (uvs.Length == pointCount)
                return (point, _) => (uint)point < (uint)uvs.Length ? uvs[point] : default;
        }
        catch { }
        return None();
    }

    // Bake local (pos, normal, uv) verts into the owner's controlling-body frame and emit a RenderMesh.
    private void Emit(List<Vtx> local, Matrix4x4 geoToWorld, string ownerPath, Vector3 color, string? texture = null, float alpha = 1f)
    {
        if (local.Count < 3) return;
        string? body = ControllingBody(ownerPath);

        // World positions (for ground-skip + framing bounds).
        Vector3 lo = new(float.MaxValue), hi = new(float.MinValue);
        foreach (Vtx v in local) { Vector3 w = Vector3.Transform(v.Pos, geoToWorld); lo = Vector3.Min(lo, w); hi = Vector3.Max(hi, w); }
        Vector3 span = hi - lo;
        if (body is null && span.Z < 0.2f && (span.X > 1.5f || span.Y > 1.5f)) return; // static ground/floor → skip
        _wMin = Vector3.Min(_wMin, lo); _wMax = Vector3.Max(_wMax, hi);

        Matrix4x4 toBaked = geoToWorld;
        if (body is not null && _bodyInverse.TryGetValue(body, out Matrix4x4 inv)) toBaked = geoToWorld * inv;

        // The texture only ships if its pixels decode; otherwise the mesh falls back to its tint.
        if (texture is not null && !EnsureTexture(texture)) texture = null;

        var verts = new float[local.Count * 8];
        for (int i = 0; i < local.Count; i++)
        {
            Vector3 pos = Vector3.Transform(local[i].Pos, toBaked);
            Vector3 nrm = Vector3.TransformNormal(local[i].Nrm, toBaked);
            nrm = nrm.LengthSquared() > 1e-12f ? Vector3.Normalize(nrm) : Vector3.UnitZ;
            int o = i * 8;
            verts[o] = pos.X; verts[o + 1] = pos.Y; verts[o + 2] = pos.Z;
            verts[o + 3] = nrm.X; verts[o + 4] = nrm.Y; verts[o + 5] = nrm.Z;
            verts[o + 6] = local[i].Uv.X; verts[o + 7] = local[i].Uv.Y;
        }
        _out.Add(new RenderMesh(verts, color, body, texture, alpha));
    }

    // Decodes an albedo file into the shared texture set (once per distinct path), downscaled to a
    // navigation-viewport-appropriate size. Returns false when the file is missing or undecodable.
    private bool EnsureTexture(string path)
    {
        if (_textures.ContainsKey(path)) return true;
        try
        {
            if (!File.Exists(path)) return false;
            const int MaxDim = 256; // nav view: tint fidelity matters, texel fidelity does not
            using var src = new System.Drawing.Bitmap(path);
            int w = src.Width, h = src.Height;
            if (Math.Max(w, h) > MaxDim)
            {
                float s = (float)MaxDim / Math.Max(w, h);
                w = Math.Max(1, (int)(w * s)); h = Math.Max(1, (int)(h * s));
            }
            using var bmp = new System.Drawing.Bitmap(src, w, h);
            var data = bmp.LockBits(new System.Drawing.Rectangle(0, 0, w, h),
                System.Drawing.Imaging.ImageLockMode.ReadOnly, System.Drawing.Imaging.PixelFormat.Format32bppArgb);
            var rgba = new byte[w * h * 4];
            unsafe
            {
                byte* srcBase = (byte*)data.Scan0;
                for (int y = 0; y < h; y++)
                {
                    // Flip rows (bitmap is top-down; GL/USD st origin is bottom-left) and BGRA → RGBA.
                    byte* row = srcBase + y * data.Stride;
                    int dst = (h - 1 - y) * w * 4;
                    for (int x = 0; x < w; x++)
                    {
                        rgba[dst + 0] = row[x * 4 + 2];
                        rgba[dst + 1] = row[x * 4 + 1];
                        rgba[dst + 2] = row[x * 4 + 0];
                        rgba[dst + 3] = row[x * 4 + 3];
                        dst += 4;
                    }
                }
            }
            bmp.UnlockBits(data);
            _textures[path] = new TextureData(w, h, rgba);
            return true;
        }
        catch { return false; }
    }

    // Standard USD display semantics the viewport must respect: purpose "guide"/"proxy" marks
    // non-render geometry (physics colliders, preview stand-ins), and authored invisibility hides a
    // subtree (e.g. helper rigid bodies like an articulation-root hack cube).
    private static bool IsRenderable(UsdPrim prim)
    {
        try
        {
            UsdAttribute p = prim.GetAttribute(new TfToken("purpose"));
            if (p.IsValid())
            {
                TfToken t = p.Get(UsdTimeCode.Default());
                string s = t?.GetString() ?? "";
                if (s is "guide" or "proxy") return false;
            }
            UsdAttribute v = prim.GetAttribute(new TfToken("visibility"));
            if (v.IsValid())
            {
                TfToken t = v.Get(UsdTimeCode.Default());
                if (t?.GetString() == "invisible") return false;
            }
        }
        catch { }
        return true;
    }

    // Walk ancestors from the prim path up toward the root, returning the nearest one that is a tracked rigid
    // body (so the mesh follows that body's live pose); null if no ancestor is a body → static world geometry.
    private string? ControllingBody(string primPath)
    {
        for (string pth = primPath; pth.Length > 1;)
        {
            if (_bodies.Contains(pth)) return pth;
            int slash = pth.LastIndexOf('/');
            if (slash <= 0) break;
            pth = pth[..slash];
        }
        return null;
    }

    // ---- look: material albedo texture → displayColor → material diffuse constant → stable hash ----

    private (Vector3 Color, string? Texture) LookFor(UsdPrim prim, string ownerPath)
    {
        MaterialLook ml = MaterialLookFor(prim);
        if (ml.Texture is not null)
        {
            // A near-black constant next to an authored texture is exporter noise, not a real tint.
            Vector3 tint = ml.Tint is { } t && t.X + t.Y + t.Z > 0.05f ? t : Vector3.One;
            return (tint, ml.Texture);
        }

        UsdAttribute dc = prim.GetAttribute(new TfToken("primvars:displayColor"));
        if (dc.IsValid())
            try
            {
                VtVec3fArray a = dc.Get(UsdTimeCode.Default());
                if (a is not null && a.size() > 0)
                {
                    GfVec3f c = a[0];
                    // Near-black displayColor is usually exporter noise, not an authored look — skip it.
                    if (c[0] + c[1] + c[2] > 0.02f) return (new Vector3(c[0], c[1], c[2]), null);
                }
            }
            catch { }

        return (ml.Tint ?? Hash(ownerPath), null);
    }

    // Follow material:binding to the bound material and cache what it contributes (per material prim:
    // many meshes share one material, and the .mdl-source probe below does file IO).
    private MaterialLook MaterialLookFor(UsdPrim prim)
    {
        try
        {
            UsdRelationship rel = prim.GetRelationship(new TfToken("material:binding"));
            if (!rel.IsValid()) return default;
            SdfPathVector targets = rel.GetTargets();
            if (targets is null || targets.Count == 0)
            {
                // A binding exists but its target didn't survive composition (payload-scope culling —
                // common in vendor assets): approximate from an enclosing Looks scope instead.
                MaterialLook fb = AncestorLooksFallback(prim.GetPath().GetString());
                if (Environment.GetEnvironmentVariable("GEMELLI_RASTER_DEBUG") == "1")
                    Console.Error.WriteLine($"[raster] culled binding on {prim.GetPath().GetString()} -> fallback tex={fb.Texture ?? "-"} tint={fb.Tint?.ToString() ?? "-"}");
                return fb;
            }
            string matPath = targets[0].GetString();
            if (_materialCache.TryGetValue(matPath, out MaterialLook cached)) return cached;

            UsdPrim mat = _stage.GetPrimAtPath(targets[0]);
            MaterialLook look = mat.IsValid() ? ScanMaterial(mat) : default;
            _materialCache[matPath] = look;
            return look;
        }
        catch { return default; }
    }

    // For a broken binding: walk up from the mesh looking for a sibling "Looks" scope and take its
    // first textured material (else first tinted). Approximate by design — the authored target was
    // culled, so the true material is unknowable; a plausible nearby texture beats a hash tint.
    private MaterialLook AncestorLooksFallback(string primPath)
    {
        for (string pth = primPath; ;)
        {
            int slash = pth.LastIndexOf('/');
            if (slash <= 0) break;
            pth = pth[..slash];
            string looksPath = pth + "/Looks";
            if (!_materialCache.TryGetValue(looksPath, out MaterialLook best))
            {
                best = default;
                UsdPrim looks = _stage.GetPrimAtPath(new SdfPath(looksPath));
                if (looks.IsValid())
                    foreach (UsdPrim m in looks.GetChildren())
                    {
                        MaterialLook l = ScanMaterial(m);
                        if (l.Texture is not null) { best = l; break; }
                        if (best.Tint is null && l.Tint is not null) best = l;
                    }
                _materialCache[looksPath] = best;
            }
            if (best.Texture is not null || best.Tint is not null) return best;
        }
        return default;
    }

    private MaterialLook ScanMaterial(UsdPrim mat)
    {
        Vector3? tint = null;
        string? texture = null;
        foreach (UsdPrim shader in mat.GetDescendants())
        {
            // Albedo texture authored as a USD input (OmniPBR and friends).
            texture ??= ReadAssetPath(shader, "inputs:diffuse_texture");

            // Constant diffuse / tint inputs (UsdPreviewSurface diffuseColor, OmniPBR constants).
            foreach (string attr in DiffuseAttrs) tint ??= ReadColor(shader, attr);
            tint ??= ReadColor(shader, "inputs:diffuse_tint");

            // Materials whose parameters live in the MDL source rather than USD inputs (e.g. the
            // conveyor's Material Library): pull the albedo path out of the .mdl module text.
            if (texture is null && ReadAssetPath(shader, "info:mdl:sourceAsset") is { } mdl)
                texture = AlbedoFromMdlSource(mdl);
        }
        return new MaterialLook(tint, texture);
    }

    // The .mdl modules used here are small text files that pass the albedo to their base material as
    //   diffuse_texture: texture_2d("../Textures/T_Steel_A1_Albedo.png", ...)
    // — enough to recover the texture without evaluating MDL.
    private static string? AlbedoFromMdlSource(string mdlPath)
    {
        try
        {
            if (!File.Exists(mdlPath) || new FileInfo(mdlPath).Length > 64 * 1024) return null;
            var m = System.Text.RegularExpressions.Regex.Match(
                File.ReadAllText(mdlPath), "diffuse_texture\\s*:\\s*texture_2d\\(\\s*\"([^\"]+)\"");
            if (!m.Success) return null;
            return Path.GetFullPath(Path.Combine(Path.GetDirectoryName(mdlPath) ?? ".", m.Groups[1].Value));
        }
        catch { return null; }
    }

    private static Vector3? ReadColor(UsdPrim prim, string attrName)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return null;
        try { GfVec3f c = a.Get(UsdTimeCode.Default()); if (c is not null) return new Vector3(c[0], c[1], c[2]); } catch { }
        return null;
    }

    // Resolved filesystem path of an asset-valued attribute (resolution handles layer-relative paths).
    private static string? ReadAssetPath(UsdPrim prim, string attrName)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attrName));
        if (!a.IsValid()) return null;
        try
        {
            SdfAssetPath ap = a.Get(UsdTimeCode.Default());
            if (ap is null) return null;
            string resolved = ap.GetResolvedPath();
            return string.IsNullOrEmpty(resolved) ? null : resolved;
        }
        catch { return null; }
    }

    // Deterministic fallback tint: FNV-1a hash the prim path, map to a hue, and return a muted pastel so
    // distinct prims get stable, visually-separable colors when no authored color exists.
    private static Vector3 Hash(string path)
    {
        uint h = 2166136261u;
        foreach (char ch in path) h = (h ^ ch) * 16777619u;
        return HsvToRgb((h % 360) / 360f, 0.22f, 0.82f);
    }

    // ---- USD shape prims → (pos, normal) verts in shape-local space ----

    private static List<Vtx>? ShapeVerts(UsdPrim prim, string type) => type switch
    {
        "Cube" => Box((float)GetDouble(prim, "size", 2.0) * 0.5f, (float)GetDouble(prim, "size", 2.0) * 0.5f, (float)GetDouble(prim, "size", 2.0) * 0.5f),
        "Sphere" => Sphere((float)GetDouble(prim, "radius", 1.0), 24, 16),
        "Cylinder" => Cylinder((float)GetDouble(prim, "radius", 0.5), (float)GetDouble(prim, "height", 1.0), 24),
        "Capsule" => Cylinder((float)GetDouble(prim, "radius", 0.5), (float)GetDouble(prim, "height", 1.0), 24),
        "Cone" => Cylinder((float)GetDouble(prim, "radius", 0.5), (float)GetDouble(prim, "height", 1.0), 24, 0f),
        "Plane" => Box((float)GetDouble(prim, "width", 1.0) * 0.5f, (float)GetDouble(prim, "length", 1.0) * 0.5f, 0.001f),
        _ => null,
    };

    // Read a scalar double attribute, returning the default if it is absent or unreadable.
    private static double GetDouble(UsdPrim prim, string attr, double dflt)
    {
        UsdAttribute a = prim.GetAttribute(new TfToken(attr));
        if (!a.IsValid()) return dflt;
        try { return a.Get(UsdTimeCode.Default()); } catch { return dflt; }
    }

    // Axis-aligned box of half-extents (hx,hy,hz) as 12 triangles with flat per-face normals (hard edges).
    private static List<Vtx> Box(float hx, float hy, float hz)
    {
        Vector3[] c =
        {
            new(-hx,-hy,-hz), new(hx,-hy,-hz), new(hx,hy,-hz), new(-hx,hy,-hz),
            new(-hx,-hy, hz), new(hx,-hy, hz), new(hx,hy, hz), new(-hx,hy, hz),
        };
        int[,] faces = { {0,3,2,1},{4,5,6,7},{0,1,5,4},{2,3,7,6},{1,2,6,5},{0,4,7,3} };
        var v = new List<Vtx>();
        for (int f = 0; f < 6; f++)
        {
            int a = faces[f, 0], b = faces[f, 1], d = faces[f, 2], e = faces[f, 3];
            Vector3 n = Vector3.Normalize(Vector3.Cross(c[b] - c[a], c[d] - c[a])); // flat per face (hard edges)
            v.Add(new(c[a], n)); v.Add(new(c[b], n)); v.Add(new(c[d], n));
            v.Add(new(c[a], n)); v.Add(new(c[d], n)); v.Add(new(c[e], n));
        }
        return v;
    }

    // UV sphere of radius r tessellated into slices×stacks quads (two tris each); radial normals → smooth shading.
    private static List<Vtx> Sphere(float r, int slices, int stacks)
    {
        var grid = new Vector3[stacks + 1, slices + 1];
        for (int i = 0; i <= stacks; i++)
        {
            float phi = MathF.PI * i / stacks;
            for (int j = 0; j <= slices; j++)
            {
                float theta = 2 * MathF.PI * j / slices;
                grid[i, j] = new Vector3(r * MathF.Sin(phi) * MathF.Cos(theta), r * MathF.Sin(phi) * MathF.Sin(theta), r * MathF.Cos(phi));
            }
        }
        var v = new List<Vtx>();
        Vtx V(Vector3 p) => new(p, Vector3.Normalize(p)); // radial normal = smooth
        for (int i = 0; i < stacks; i++)
            for (int j = 0; j < slices; j++)
            {
                v.Add(V(grid[i, j])); v.Add(V(grid[i + 1, j])); v.Add(V(grid[i + 1, j + 1]));
                v.Add(V(grid[i, j])); v.Add(V(grid[i + 1, j + 1])); v.Add(V(grid[i, j + 1]));
            }
        return v;
    }

    // Z-aligned cylinder (radius r, height h) with smooth radial side normals and two flat caps. A NaN
    // topRadius means a straight cylinder; passing topRadius=0 collapses the top to a point → a cone.
    private static List<Vtx> Cylinder(float r, float h, int slices, float topRadius = float.NaN)
    {
        float rt = float.IsNaN(topRadius) ? r : topRadius;
        float z0 = -h * 0.5f, z1 = h * 0.5f;
        var v = new List<Vtx>();
        for (int j = 0; j < slices; j++)
        {
            float a0 = 2 * MathF.PI * j / slices, a1 = 2 * MathF.PI * (j + 1) / slices;
            Vector3 nb0 = new(MathF.Cos(a0), MathF.Sin(a0), 0), nb1 = new(MathF.Cos(a1), MathF.Sin(a1), 0); // side: radial smooth
            Vector3 b0 = new(r * nb0.X, r * nb0.Y, z0), b1 = new(r * nb1.X, r * nb1.Y, z0);
            Vector3 t0 = new(rt * nb0.X, rt * nb0.Y, z1), t1 = new(rt * nb1.X, rt * nb1.Y, z1);
            v.Add(new(b0, nb0)); v.Add(new(b1, nb1)); v.Add(new(t1, nb1));
            v.Add(new(b0, nb0)); v.Add(new(t1, nb1)); v.Add(new(t0, nb0));
            Vector3 dn = -Vector3.UnitZ, un = Vector3.UnitZ;
            v.Add(new(new Vector3(0, 0, z0), dn)); v.Add(new(b1, dn)); v.Add(new(b0, dn)); // bottom cap (flat)
            v.Add(new(new Vector3(0, 0, z1), un)); v.Add(new(t0, un)); v.Add(new(t1, un)); // top cap (flat)
        }
        return v;
    }

    // Standard HSV→RGB conversion (hue h in [0,1)); used only by the hash-tint fallback.
    private static Vector3 HsvToRgb(float h, float s, float vv)
    {
        float i = MathF.Floor(h * 6f), f = h * 6f - i;
        float p = vv * (1 - s), q = vv * (1 - f * s), t = vv * (1 - (1 - f) * s);
        return ((int)i % 6) switch
        {
            0 => new(vv, t, p), 1 => new(q, vv, p), 2 => new(p, vv, t),
            3 => new(p, q, vv), 4 => new(t, p, vv), _ => new(vv, p, q),
        };
    }

    // Rotation + translation only (scale dropped), matching the rigid physics pose convention.
    private static Matrix4x4 RigidPart(Matrix4x4 m)
    {
        if (!Matrix4x4.Decompose(m, out _, out Quaternion q, out Vector3 t)) return m;
        Matrix4x4 r = Matrix4x4.CreateFromQuaternion(q);
        r.M41 = t.X; r.M42 = t.Y; r.M43 = t.Z;
        return r;
    }

    // GfMatrix4d (row-vector) → System.Numerics.Matrix4x4 (also row-vector — same convention).
    private static Matrix4x4 ToM(GfMatrix4d m) => new(
        (float)m.GetRow(0)[0], (float)m.GetRow(0)[1], (float)m.GetRow(0)[2], (float)m.GetRow(0)[3],
        (float)m.GetRow(1)[0], (float)m.GetRow(1)[1], (float)m.GetRow(1)[2], (float)m.GetRow(1)[3],
        (float)m.GetRow(2)[0], (float)m.GetRow(2)[1], (float)m.GetRow(2)[2], (float)m.GetRow(2)[3],
        (float)m.GetRow(3)[0], (float)m.GetRow(3)[1], (float)m.GetRow(3)[2], (float)m.GetRow(3)[3]);
}
