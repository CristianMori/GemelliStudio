using Gemelli.Core;
using Gemelli.Core.Ipc;
using Gemelli.Physics;

// ovGemelli physics worker: hosts ovphysx alone and serves PhysicsOp commands over a named pipe.
// Launched by the TwinSession orchestrator with the pipe name as argv[0] and OVPHYSX_LIB in the env.

CrashDialogs.Suppress();

if (args.Length < 1)
{
    Console.Error.WriteLine("usage: Gemelli.PhysicsHost <pipe-name>");
    return 2;
}

string pipeName = args[0];
using var worker = new PhysicsWorker();
try
{
    new PipeServer(pipeName).Run(worker.Dispatch);
    return 0;
}
catch (Exception ex)
{
    Console.Error.WriteLine($"[physics-host] fatal: {ex}");
    return 1;
}
