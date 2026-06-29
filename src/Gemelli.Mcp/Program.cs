using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using Gemelli.Core;
using Gemelli.Core.Control;

// Gemelli MCP server: exposes the digital twin as callable tools over the Model Context Protocol (stdio).
// Drives one shared TwinService (which owns the two-process twin). All logging goes to stderr so
// stdout stays pure MCP/JSON-RPC; worker stdout/stderr is already captured by WorkerHost.
//
// Configure in any MCP-compatible client by pointing it at the built exe; the native libraries are
// auto-discovered under native/, or set OVPHYSX_LIB and GEMELLI_OVRTX_DIR in the client's env (see README).

CrashDialogs.Suppress();

var builder = Host.CreateApplicationBuilder(args);

builder.Logging.AddConsole(o => o.LogToStandardErrorThreshold = LogLevel.Trace);

// One twin per server process, shared by all tool calls (TwinService serializes onto its sim thread).
builder.Services.AddSingleton<TwinService>();

builder.Services
    .AddMcpServer()
    .WithStdioServerTransport()
    .WithToolsFromAssembly();

await builder.Build().RunAsync();
