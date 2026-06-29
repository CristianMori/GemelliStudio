using Gemelli.Core;
using Gemelli.Core.Ipc;
using Gemelli.Render;

// ovGemelli render worker: hosts ovrtx alone and serves RenderOp commands over a named pipe.
// Launched by the TwinSession orchestrator with the pipe name as argv[0].

CrashDialogs.Suppress();

if (args.Length < 1)
{
    Console.Error.WriteLine("usage: Gemelli.RenderHost <pipe-name>");
    return 2;
}

string pipeName = args[0];
using var worker = new RenderWorker();
try
{
    new PipeServer(pipeName).Run(worker.Dispatch);
    return 0;
}
catch (Exception ex)
{
    Console.Error.WriteLine($"[render-host] fatal: {ex}");
    return 1;
}
