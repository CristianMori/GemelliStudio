# Full MCP agent-loop test (SEQUENTIAL, like a real agent): initialize -> start_twin -> step ->
# render_frame, waiting for each response before sending the next. Decodes the returned image.
param(
    [string]$Exe = "C:\DataDrive\ovGemelli\src\Gemelli.Mcp\bin\x64\Release\net10.0\Gemelli.Mcp.exe",
    [string]$Usd = "C:\DataDrive\ovGemelli\out\boxes_with_camera.usda",
    [string]$Product = "/Render/OmniverseKit/HydraTextures/omni_kit_widget_viewport_ViewportTexture_0",
    [string]$OutPng = "C:\DataDrive\ovGemelli\out\mcp_render.png"
)

$env:OVPHYSX_LIB = "C:\DataDrive\ovGemelli\native\ovphysx\ovphysx\lib\ovphysx.dll"
$env:GEMELLI_OVRTX_DIR = "C:\DataDrive\ovGemelli\native\ovrtx\bin"

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $Exe
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.UseShellExecute = $false
$p = [System.Diagnostics.Process]::Start($psi)
$errTask = $p.StandardError.ReadToEndAsync()  # drain stderr so it can't block; collected at end

function Send($obj) { $p.StandardInput.WriteLine(($obj | ConvertTo-Json -Depth 8 -Compress)); $p.StandardInput.Flush() }

# Read lines until a response with the given id arrives (skipping notifications/other ids). Returns the message.
function RecvId([int]$id, [int]$timeoutSec) {
    $deadline = (Get-Date).AddSeconds($timeoutSec)
    while ((Get-Date) -lt $deadline) {
        $line = $p.StandardOutput.ReadLine()
        if ($null -eq $line) { Start-Sleep -Milliseconds 30; continue }
        try { $m = $line | ConvertFrom-Json } catch { continue }
        if ($null -ne $m.id -and [int]$m.id -eq $id) { return $m }
    }
    return $null
}

function ToolText($m) { if ($m.result) { return $m.result.content[0].text } elseif ($m.error) { return "JSONRPC ERROR: " + $m.error.message } else { return "(no result)" } }

Send @{ jsonrpc="2.0"; id=1; method="initialize"; params=@{ protocolVersion="2024-11-05"; capabilities=@{}; clientInfo=@{ name="drive"; version="1.0" } } }
$null = RecvId 1 30
Send @{ jsonrpc="2.0"; method="notifications/initialized" }

Send @{ jsonrpc="2.0"; id=2; method="tools/call"; params=@{ name="start_twin"; arguments=@{ usd=$Usd; renderProducts=@($Product); device="cpu" } } }
$r2 = RecvId 2 150
Write-Output ("start_twin: " + (ToolText $r2))

Send @{ jsonrpc="2.0"; id=3; method="tools/call"; params=@{ name="step"; arguments=@{ n=30 } } }
$r3 = RecvId 3 120
Write-Output ("step: " + (ToolText $r3))

Send @{ jsonrpc="2.0"; id=4; method="tools/call"; params=@{ name="render_frame"; arguments=@{} } }
$r4 = RecvId 4 120

$p.StandardInput.Close()
try { $p.Kill() } catch {}

if ($null -eq $r4) { Write-Output "NO render_frame RESPONSE"; Write-Output ("STDERR:`n" + $errTask.Result); exit 1 }
$content = $r4.result.content[0]
Write-Output ("render_frame: type=" + $content.type + " mime=" + $content.mimeType + " isError=" + $r4.result.isError)
if ($content.type -eq "image" -and $content.data) {
    $bytes = [Convert]::FromBase64String($content.data)
    [IO.File]::WriteAllBytes($OutPng, $bytes)
    Write-Output ("Saved " + $bytes.Length + " bytes -> " + $OutPng)
} else {
    Write-Output ("render_frame text: " + $content.text)
    Write-Output ("STDERR tail:`n" + (($errTask.Result -split "`n") | Select-Object -Last 15 | Out-String))
}
