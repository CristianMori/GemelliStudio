using System.Collections.Concurrent;
using System.Globalization;
using System.Text;
using Gemelli.Core.Imaging;

namespace Gemelli.Core.Sensors;

/// <summary>
/// Records a sensor product's frames to disk as a labeled synthetic-data set: color PNG, depth as both a
/// normalized preview PNG and raw float32 <c>.f32</c>, segmentation PNG when present, plus a
/// <c>manifest.jsonl</c> with per-frame timing. Frames are queued and written on a background thread so
/// the simulation loop never blocks on disk IO; if the writer falls behind, frames are dropped (counted).
/// </summary>
public sealed class SensorRecorder : IDisposable
{
    private readonly BlockingCollection<Item> _queue = new(boundedCapacity: 64);
    private readonly Thread _worker;
    private readonly string _dir;
    private readonly StreamWriter _manifest;
    private int _written;
    private int _dropped;
    private bool _disposed;

    private readonly record struct Item(int Index, CapturedFrame Frame);

    /// <summary>Creates the output directory, opens the manifest, and starts the background writer thread.</summary>
    public SensorRecorder(string directory)
    {
        _dir = directory;
        System.IO.Directory.CreateDirectory(_dir);
        _manifest = new StreamWriter(Path.Combine(_dir, "manifest.jsonl"), append: false, Encoding.UTF8);
        _worker = new Thread(Drain) { IsBackground = true, Name = "gemelli-recorder" };
        _worker.Start();
    }

    public string Directory => _dir;
    public int Written => Volatile.Read(ref _written);
    public int Dropped => Volatile.Read(ref _dropped);

    /// <summary>Queue a frame for writing. Non-blocking; drops (and counts) the frame if the queue is full.</summary>
    public void Submit(int index, CapturedFrame frame)
    {
        if (_disposed) return;
        if (!_queue.TryAdd(new Item(index, frame)))
            Interlocked.Increment(ref _dropped);
    }

    /// <summary>Worker loop: pulls queued frames and writes each, swallowing per-frame failures.</summary>
    private void Drain()
    {
        foreach (Item item in _queue.GetConsumingEnumerable())
        {
            try { WriteOne(item.Index, item.Frame); }
            catch { /* never let a bad frame kill the recorder */ }
        }
    }

    /// <summary>Writes one frame's available channels (color/depth/seg) to disk and appends a manifest line.</summary>
    private void WriteOne(int index, CapturedFrame frame)
    {
        string stem = index.ToString("D5", CultureInfo.InvariantCulture);
        var sb = new StringBuilder();
        sb.Append("{\"frame\":").Append(index)
          .Append(",\"t0\":").Append(frame.StartTime.ToString("R", CultureInfo.InvariantCulture))
          .Append(",\"t1\":").Append(frame.EndTime.ToString("R", CultureInfo.InvariantCulture));

        if (SensorVisualize.ColorRgba(frame.Color) is { } c)
        {
            File.WriteAllBytes(Path.Combine(_dir, $"color_{stem}.png"), Png.Encode(c.Item3, c.Item1, c.Item2, 4));
            sb.Append(",\"color\":\"color_").Append(stem).Append(".png\",\"width\":").Append(c.Item1).Append(",\"height\":").Append(c.Item2);
        }

        if (frame.Depth is { } depth)
        {
            // Raw float32 distance (reinterpreted to float regardless of stored dtype) for downstream use.
            File.WriteAllBytes(Path.Combine(_dir, $"depth_{stem}.f32"), depth.Bytes);
            if (SensorVisualize.DepthGray(depth) is { } dg)
                File.WriteAllBytes(Path.Combine(_dir, $"depth_{stem}.png"), Png.Encode(dg.Item3, dg.Item1, dg.Item2, 4));
            sb.Append(",\"depth\":\"depth_").Append(stem).Append(".f32\"");
        }

        if (frame.Segmentation is { } seg)
        {
            File.WriteAllBytes(Path.Combine(_dir, $"seg_{stem}.u32"), seg.Bytes);
            if (SensorVisualize.SegmentationColor(seg) is { } sg)
                File.WriteAllBytes(Path.Combine(_dir, $"seg_{stem}.png"), Png.Encode(sg.Item3, sg.Item1, sg.Item2, 4));
            sb.Append(",\"seg\":\"seg_").Append(stem).Append(".u32\"");
        }

        sb.Append('}');
        lock (_manifest) _manifest.WriteLine(sb.ToString());
        Interlocked.Increment(ref _written);
    }

    /// <summary>Stops accepting frames, drains the writer (5s grace), then flushes and closes the manifest.</summary>
    public void Dispose()
    {
        if (_disposed) return;
        _disposed = true;
        _queue.CompleteAdding();
        _worker.Join(TimeSpan.FromSeconds(5));
        try { lock (_manifest) { _manifest.Flush(); _manifest.Dispose(); } } catch { }
        _queue.Dispose();
    }
}
