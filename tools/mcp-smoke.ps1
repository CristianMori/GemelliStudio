# MCP stdio smoke test: start the server, send initialize + tools/list, print the tool names.
param([string]$Exe = "C:\DataDrive\ovGemelli\src\Gemelli.Mcp\bin\x64\Release\net10.0\Gemelli.Mcp.exe")

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $Exe
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.UseShellExecute = $false
$p = [System.Diagnostics.Process]::Start($psi)

$init = '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2024-11-05","capabilities":{},"clientInfo":{"name":"smoke","version":"1.0"}}}'
$inited = '{"jsonrpc":"2.0","method":"notifications/initialized"}'
$list = '{"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}'

$p.StandardInput.WriteLine($init)
$p.StandardInput.WriteLine($inited)
$p.StandardInput.WriteLine($list)
$p.StandardInput.Flush()

# Read stdout lines until we get the tools/list response (id 2) or time out.
$deadline = (Get-Date).AddSeconds(20)
$toolsLine = $null
while ((Get-Date) -lt $deadline) {
    $line = $p.StandardOutput.ReadLine()
    if ($null -eq $line) { Start-Sleep -Milliseconds 50; continue }
    if ($line -match '"id":2') { $toolsLine = $line; break }
}

$p.StandardInput.Close()
try { $p.Kill() } catch {}

if ($null -eq $toolsLine) {
    Write-Output "NO tools/list RESPONSE"
    Write-Output ("STDERR: " + $p.StandardError.ReadToEnd())
    exit 1
}

$resp = $toolsLine | ConvertFrom-Json
$names = $resp.result.tools | ForEach-Object { $_.name }
Write-Output ("TOOLS: " + ($names -join ", "))
Write-Output ("COUNT: " + $names.Count)
