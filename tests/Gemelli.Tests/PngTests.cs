using System.Buffers.Binary;
using System.IO.Compression;
using Gemelli.Core.Imaging;
using Xunit;

namespace Gemelli.Tests;

/// <summary>
/// Tier-1 tests for the hand-rolled PNG encoder: full structural decode (signature, chunk layout,
/// IHDR fields, per-chunk CRC-32 against an independent bit-by-bit implementation) and a pixel
/// round-trip through the zlib-inflated IDAT scanlines. A silent bug here corrupts every recorded
/// dataset image and MCP screenshot, so the file is verified byte-for-byte, not just "non-empty".
/// </summary>
public class PngTests
{
    [Fact]
    public void Encodes_Rgb_With_Valid_Structure_And_Pixels_Round_Trip()
    {
        var (w, h, ch) = (3, 2, 3);
        byte[] pixels = MakePixels(w, h, ch);
        AssertValidPng(Png.Encode(pixels, w, h, ch), w, h, ch, pixels);
    }

    [Fact]
    public void Encodes_Rgba_With_Valid_Structure_And_Pixels_Round_Trip()
    {
        var (w, h, ch) = (5, 4, 4);
        byte[] pixels = MakePixels(w, h, ch);
        AssertValidPng(Png.Encode(pixels, w, h, ch), w, h, ch, pixels);
    }

    [Fact]
    public void Encodes_A_Single_Pixel()
    {
        byte[] pixels = [1, 2, 3];
        AssertValidPng(Png.Encode(pixels, 1, 1, 3), 1, 1, 3, pixels);
    }

    [Fact]
    public void Rejects_Unsupported_Channel_Counts_And_Short_Buffers()
    {
        Assert.Throws<ArgumentException>(() => Png.Encode(new byte[8], 2, 2, 2));  // 2 channels
        Assert.Throws<ArgumentException>(() => Png.Encode(new byte[11], 2, 2, 3)); // needs 12 bytes
    }

    // Deterministic non-uniform pixel pattern so a row-order or stride bug shows up.
    private static byte[] MakePixels(int w, int h, int ch)
    {
        var p = new byte[w * h * ch];
        for (int i = 0; i < p.Length; i++) p[i] = (byte)(i * 7 + 13);
        return p;
    }

    // Full structural verification: signature, chunk sequence, IHDR contents, every chunk's CRC,
    // then inflate IDAT and compare filter bytes + scanline pixels against the source buffer.
    private static void AssertValidPng(byte[] png, int width, int height, int channels, byte[] pixels)
    {
        byte[] signature = [137, 80, 78, 71, 13, 10, 26, 10];
        Assert.Equal(signature, png.Take(8));

        int pos = 8;
        var chunks = new List<(string Type, byte[] Data)>();
        while (pos < png.Length)
        {
            int len = BinaryPrimitives.ReadInt32BigEndian(png.AsSpan(pos, 4));
            string type = System.Text.Encoding.ASCII.GetString(png, pos + 4, 4);
            byte[] data = png.AsSpan(pos + 8, len).ToArray();
            uint storedCrc = BinaryPrimitives.ReadUInt32BigEndian(png.AsSpan(pos + 8 + len, 4));

            // CRC over type+data, recomputed with an independent bit-by-bit implementation (the
            // encoder is table-driven; a shared bug can't cancel out).
            Assert.Equal(ReferenceCrc32(png.AsSpan(pos + 4, 4 + len)), storedCrc);

            chunks.Add((type, data));
            pos += 12 + len;
        }
        Assert.Equal(png.Length, pos); // no trailing garbage

        Assert.Equal(["IHDR", "IDAT", "IEND"], chunks.Select(c => c.Type));

        byte[] ihdr = chunks[0].Data;
        Assert.Equal(13, ihdr.Length);
        Assert.Equal(width, BinaryPrimitives.ReadInt32BigEndian(ihdr.AsSpan(0, 4)));
        Assert.Equal(height, BinaryPrimitives.ReadInt32BigEndian(ihdr.AsSpan(4, 4)));
        Assert.Equal(8, ihdr[8]);                                // bit depth
        Assert.Equal(channels == 4 ? 6 : 2, ihdr[9]);            // color type: RGBA / RGB
        Assert.Equal(0, ihdr[10]); Assert.Equal(0, ihdr[11]); Assert.Equal(0, ihdr[12]);
        Assert.Empty(chunks[2].Data);                            // IEND carries no payload

        // Inflate IDAT (zlib-wrapped DEFLATE) and verify scanlines: [filter 0][row pixels] per row.
        using var inflated = new MemoryStream();
        using (var z = new ZLibStream(new MemoryStream(chunks[1].Data), CompressionMode.Decompress))
            z.CopyTo(inflated);
        byte[] raw = inflated.ToArray();

        int stride = width * channels;
        Assert.Equal((stride + 1) * height, raw.Length);
        for (int y = 0; y < height; y++)
        {
            Assert.Equal(0, raw[y * (stride + 1)]); // filter byte: None
            Assert.Equal(pixels.AsSpan(y * stride, stride), raw.AsSpan(y * (stride + 1) + 1, stride));
        }
    }

    // Independent CRC-32 (PNG polynomial), bit-by-bit — deliberately not table-driven.
    private static uint ReferenceCrc32(ReadOnlySpan<byte> data)
    {
        uint crc = 0xFFFFFFFF;
        foreach (byte b in data)
        {
            crc ^= b;
            for (int k = 0; k < 8; k++)
                crc = (crc & 1) != 0 ? 0xEDB88320 ^ (crc >> 1) : crc >> 1;
        }
        return crc ^ 0xFFFFFFFF;
    }
}
