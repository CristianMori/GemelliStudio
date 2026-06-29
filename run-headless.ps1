# Run the Gemelli headless host (no UI). The app builds into dist\<Config>\ (Headless in the root, the
# physics/render workers in their own subfolders beside it). Native libraries are auto-discovered under
# native\. All arguments are passed straight through to Gemelli.Headless.
#
#   .\run-headless.ps1 --usd scenes\franka_studio.usda --products /Render/OmniverseKit/HydraTextures/camera_sensor_162912244368 --steps 60 --device gpu
#   .\run-headless.ps1 --usd <scene> --products <product> --record out\dataset   # record color+depth+seg dataset
#
# See the full flag list with no arguments. Use -Debug to run the Debug build.
param([switch]$Debug)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$config = if ($Debug) { 'Debug' } else { 'Release' }
$exe = Join-Path $root "dist\$config\Gemelli.Headless.exe"

if (-not (Test-Path $exe)) {
    Write-Host "Building Gemelli ($config, x64)..." -ForegroundColor Cyan
    dotnet build (Join-Path $root 'Gemelli.slnx') -c $config --nologo
    if ($LASTEXITCODE -ne 0) { Write-Error 'Build failed.'; exit 1 }
}
if (-not (Test-Path $exe)) { Write-Error "Could not locate $exe after build."; exit 1 }

& $exe @args
exit $LASTEXITCODE
