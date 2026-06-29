using System.Numerics;
using Gemelli.Core.Imaging;
using Gemelli.Viewport;

// De-risk probe: load a scene's geometry, rasterize one frame offscreen, save a PNG.
// All geometry is baked to world (load-time pose), so the robot appears in its default config.
// Usage: raster-probe <usd> [out.png]

string usd = args.Length > 0 ? args[0] : @"C:\DataDrive\ovGemelli\scenes\franka_studio.usda";
string outPng = args.Length > 1 ? args[1] : @"C:\DataDrive\ovGemelli\out\raster.png";
const int W = 1280, H = 720;

GeometryResult geo = UsdGeometryLoader.Load(usd, System.Array.Empty<string>());
List<RenderMesh> meshes = geo.Meshes;
Console.WriteLine($"Loaded {meshes.Count} meshes, {meshes.Sum(m => m.Vertices.Length / 18)} triangles.");

Vector3 center = geo.Center;
float radius = geo.Radius;
Vector3 eye = center + new Vector3(radius * 1.0f, -radius * 1.5f, radius * 0.9f);
Vector3 forward = Vector3.Normalize(center - eye);

Matrix4x4 view = ViewportCamera.View(eye, forward);
Matrix4x4 proj = ViewportCamera.Projection(0.73f, (float)W / H);

using var ras = new GlRasterizer(W, H);
ras.Upload(meshes);
byte[] rgba = ras.Render(view, proj, _ => null);

Directory.CreateDirectory(Path.GetDirectoryName(outPng)!);
File.WriteAllBytes(outPng, Png.Encode(rgba, W, H, 4));
Console.WriteLine($"Wrote {outPng}");
return 0;
