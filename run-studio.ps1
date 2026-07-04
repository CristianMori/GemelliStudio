# Launch Gemelli Studio. The whole app builds into one folder (dist\<Config>\): Studio in the root, the
# physics/render workers in their own subfolders beside it. Native libraries (ovphysx / ovrtx) are
# auto-discovered under native\, so no environment setup is needed when running from the repo. Defaults to
# a Release build (much faster viewport than Debug); builds first if the binary doesn't exist yet.
#
#   .\run-studio.ps1            # incremental Release build, then launch
#   .\run-studio.ps1 -NoBuild   # skip the build; run the binary already in dist\
#   .\run-studio.ps1 -Debug     # use the Debug build instead
param([switch]$Debug, [switch]$NoBuild)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$config = if ($Debug) { 'Debug' } else { 'Release' }
$exe = Join-Path $root "dist\$config\Gemelli.Studio.exe"

# Always build (incremental, seconds when clean) so a source edit never silently runs a stale binary.
if (-not $NoBuild) {
    dotnet build (Join-Path $root 'Gemelli.slnx') -c $config --nologo -v quiet
    if ($LASTEXITCODE -ne 0) { Write-Host 'Build failed.' -ForegroundColor Red; exit 1 }
}
if (-not (Test-Path $exe)) { Write-Host "Could not locate $exe after build." -ForegroundColor Red; exit 1 }

# Sanity-check the native libraries so the user gets a clear message before the window opens.
$physx = Join-Path $root 'native\ovphysx\ovphysx\lib\ovphysx.dll'
$ovrtx = Join-Path $root 'native\ovrtx\bin\ovrtx-dynamic.dll'
if (-not (Test-Path $physx)) { Write-Warning "ovphysx.dll not found at $physx - set OVPHYSX_LIB or place it there." }
if (-not (Test-Path $ovrtx)) { Write-Warning "ovrtx-dynamic.dll not found at $ovrtx - set GEMELLI_OVRTX_DIR or place it there." }

Write-Host "Launching $exe" -ForegroundColor Green
Start-Process -FilePath $exe
