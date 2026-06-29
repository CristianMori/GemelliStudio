using System.Runtime.InteropServices;
using Nvidia.Ovrtx;
using Nvidia.Ovrtx.Interop;
using Gemelli.Core.Imaging;

// ovrtx-ONLY render smoke (no ovphysx). Renders a USD and reports whether the image is non-black.
// Usage: ovrtx-smoke <ovrtxBin> <usd> <renderProduct> <outPng>
// Defaults render the known-good robot-ovrtx.usda from S3 (the minimal example's scene).

// Suppress Windows hard-error dialogs (SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX) so a native
// fault in ovrtx fails fast instead of blocking the process on a modal popup.
SetErrorMode(0x0001 | 0x0002);
[DllImport("kernel32.dll")] static extern uint SetErrorMode(uint mode);

string ovrtxBin = args.Length > 0 ? args[0] : @"C:\DataDrive\ovGemelli\native\ovrtx\bin";
string usd = args.Length > 1 ? args[1] : "https://omniverse-content-production.s3.us-west-2.amazonaws.com/Samples/Robot-OVRTX/robot-ovrtx.usda";
string product = args.Length > 2 ? args[2] : "/Render/Camera";
string outPng = args.Length > 3 ? args[3] : @"C:\DataDrive\ovGemelli\out\ovrtx_only.png";

OvrtxLibrary.SetLibraryPath(ovrtxBin);
Console.WriteLine($"ovrtx {Renderer.Version}; usd={usd}; product={product}");

using var renderer = new Renderer(new RendererConfig { SyncMode = true });
renderer.AddUsd(usd);
Console.WriteLine("USD loaded; warming up...");

for (int i = 0; i < 8; i++)
    using (renderer.Step([product], 0.0)) { }

using RenderProductSetOutputs outputs = renderer.Step([product], 1.0 / 60.0);
Console.WriteLine($"products: {string.Join(", ", outputs.Products.Keys)}");

foreach (var (_, prod) in outputs.Products)
foreach (var frame in prod.Frames)
{
    if (!frame.RenderVars.TryGetValue("LdrColor", out var ldr))
    {
        Console.WriteLine($"  no LdrColor; vars=[{string.Join(",", frame.RenderVars.Keys)}]");
        continue;
    }

    using MappedRenderVar mapped = ldr.Map(MapDeviceType.Cpu);
    DLTensor t = mapped.Tensor;
    var shape = t.GetShape();
    int h = (int)shape[0], w = (int)shape[1];
    int channels = Math.Max(1, (int)t.DType.Lanes);
    long bytes = (long)h * w * channels;

    var buf = new byte[bytes];
    unsafe { new ReadOnlySpan<byte>((void*)t.Data, (int)bytes).CopyTo(buf); }

    // Is it black? Max over RGB channels.
    int maxRgb = 0;
    for (long i = 0; i < bytes; i += channels)
        for (int c = 0; c < Math.Min(3, channels); c++)
            maxRgb = Math.Max(maxRgb, buf[i + c]);

    Directory.CreateDirectory(Path.GetDirectoryName(outPng)!);
    File.WriteAllBytes(outPng, Png.Encode(buf, w, h, channels));
    Console.WriteLine($"  {w}x{h}x{channels}, maxRGB={maxRgb} ({(maxRgb == 0 ? "ALL BLACK" : "has content")}) -> {outPng}");
}

Console.WriteLine("Done.");
