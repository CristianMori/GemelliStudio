# Run the Gemelli headless host (no UI). The app builds into dist\<Config>\ (Headless in the root, the
# physics/render workers in their own subfolders beside it). Native libraries are auto-discovered under
# native\. All arguments are passed straight through to Gemelli.Headless.
#
#   .\run-headless.ps1 --usd scenes\franka_studio.usda --products /Render/OmniverseKit/HydraTextures/camera_sensor_162912244368 --steps 60 --device gpu
#   .\run-headless.ps1 --usd <scene> --products <product> --record out\dataset   # record color+depth+seg dataset
#
# See the full flag list with no arguments. Use -Debug to run the Debug build; -NoBuild to skip the
# (incremental) build and run whatever binary is already in dist\.
param([switch]$Debug, [switch]$NoBuild)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$config = if ($Debug) { 'Debug' } else { 'Release' }
$exe = Join-Path $root "dist\$config\Gemelli.Headless.exe"

# Always build (incremental, seconds when clean) so a source edit never silently runs a stale binary.
if (-not $NoBuild) {
    dotnet build (Join-Path $root 'Gemelli.slnx') -c $config --nologo -v quiet
    if ($LASTEXITCODE -ne 0) { Write-Host 'Build failed.' -ForegroundColor Red; exit 1 }
}
if (-not (Test-Path $exe)) { Write-Host "Could not locate $exe after build." -ForegroundColor Red; exit 1 }

& $exe @args
exit $LASTEXITCODE
