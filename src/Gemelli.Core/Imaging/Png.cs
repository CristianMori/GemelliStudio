using System.Buffers.Binary;
using System.IO.Compression;

namespace Gemelli.Core.Imaging;

/// <summary>
/// Minimal dependency-free PNG encoder for 8-bit RGB/RGBA buffers. Avoids pulling in a licensed
/// imaging library (the same reasoning as IsaacSimSharp's bundled encoder). Uses
/// <see cref="ZLibStream"/> for the zlib-wrapped DEFLATE stream PNG requires.
/// </summary>
public static class Png
{
    private static readonly byte[] Signature = [137, 80, 78, 71, 13, 10, 26, 10];

    /// <summary>Encodes an 8-bit-per-channel image (channels = 3 for RGB, 4 for RGBA) to PNG bytes.</summary>
    public static byte[] Encode(ReadOnlySpan<byte> pixels, int width, int height, int channels)
    {
        if (channels is not (3 or 4))
            throw new ArgumentException("Only 3 (RGB) or 4 (RGBA) channels are supported.", nameof(channels));
        long expected = (long)width * height * channels;
        if (pixels.Length < expected)
            throw new ArgumentException($"pixels buffer ({pixels.Length}) smaller than width*height*channels ({expected}).", nameof(pixels));

        byte colorType = (byte)(channels == 4 ? 6 : 2); // 6 = RGBA, 2 = RGB
        using var ms = new MemoryStream();
        ms.Write(Signature);

        // IHDR
        Span<byte> ihdr = stackalloc byte[13];
        BinaryPrimitives.WriteInt32BigEndian(ihdr[..4], width);
        BinaryPrimitives.WriteInt32BigEndian(ihdr.Slice(4, 4), height);
        ihdr[8] = 8;          // bit depth
        ihdr[9] = colorType;
        ihdr[10] = 0;         // compression
        ihdr[11] = 0;         // filter
        ihdr[12] = 0;         // interlace
        WriteChunk(ms, "IHDR", ihdr);

        // IDAT: each scanline prefixed with filter byte 0 (None), then zlib-compressed.
        int stride = width * channels;
        using (var compressed = new MemoryStream())
        {
            using (var zlib = new ZLibStream(compressed, CompressionLevel.Optimal, leaveOpen: true))
            {
                var filter = new byte[1];
                for (int y = 0; y < height; y++)
                {
                    zlib.Write(filter);
                    zlib.Write(pixels.Slice(y * stride, stride));
                }
            }
            WriteChunk(ms, "IDAT", compressed.GetBuffer().AsSpan(0, (int)compressed.Length));
        }

        WriteChunk(ms, "IEND", ReadOnlySpan<byte>.Empty);
        return ms.ToArray();
    }

    /// <summary>Emits a PNG chunk: big-endian length, 4-byte type tag, payload, then CRC over type+data.</summary>
    private static void WriteChunk(Stream output, string type, ReadOnlySpan<byte> data)
    {
        Span<byte> len = stackalloc byte[4];
        BinaryPrimitives.WriteInt32BigEndian(len, data.Length);
        output.Write(len);

        Span<byte> typeBytes = stackalloc byte[4];
        for (int i = 0; i < 4; i++) typeBytes[i] = (byte)type[i];
        output.Write(typeBytes);
        output.Write(data);

        uint crc = Crc32.Compute(typeBytes);
        crc = Crc32.Compute(data, crc);
        Span<byte> crcBytes = stackalloc byte[4];
        BinaryPrimitives.WriteUInt32BigEndian(crcBytes, crc);
        output.Write(crcBytes);
    }
}

/// <summary>CRC-32 (PNG/zlib polynomial 0xEDB88320), table-driven.</summary>
internal static class Crc32
{
    private static readonly uint[] Table = BuildTable();

    /// <summary>Precomputes the 256-entry lookup table for the reflected CRC-32 polynomial.</summary>
    private static uint[] BuildTable()
    {
        var table = new uint[256];
        for (uint n = 0; n < 256; n++)
        {
            uint c = n;
            for (int k = 0; k < 8; k++)
                c = (c & 1) != 0 ? 0xEDB88320 ^ (c >> 1) : c >> 1;
            table[n] = c;
        }
        return table;
    }

    /// <summary>Accumulates CRC-32 over <paramref name="data"/>; pass a prior result as <paramref name="seed"/> to chain spans.</summary>
    public static uint Compute(ReadOnlySpan<byte> data, uint seed = 0)
    {
        uint c = seed ^ 0xFFFFFFFF;
        foreach (byte b in data)
            c = Table[(c ^ b) & 0xFF] ^ (c >> 8);
        return c ^ 0xFFFFFFFF;
    }
}
