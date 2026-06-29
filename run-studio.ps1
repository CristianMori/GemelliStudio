# Launch Gemelli Studio. The whole app builds into one folder (dist\<Config>\): Studio in the root, the
# physics/render workers in their own subfolders beside it. Native libraries (ovphysx / ovrtx) are
# auto-discovered under native\, so no environment setup is needed when running from the repo. Defaults to
# a Release build (much faster viewport than Debug); builds first if the binary doesn't exist yet.
#
#   .\run-studio.ps1            # Release: build if missing, then launch
#   .\run-studio.ps1 -Build     # force a fresh Release build, then launch
#   .\run-studio.ps1 -Debug     # use the Debug build instead
param([switch]$Build, [switch]$Debug)

$ErrorActionPreference = 'Stop'
$root = $PSScriptRoot
$config = if ($Debug) { 'Debug' } else { 'Release' }
$exe = Join-Path $root "dist\$config\Gemelli.Studio.exe"

if ($Build -or -not (Test-Path $exe)) {
    Write-Host "Building Gemelli ($config, x64)..." -ForegroundColor Cyan
    dotnet build (Join-Path $root 'Gemelli.slnx') -c $config --nologo
    if ($LASTEXITCODE -ne 0) { Write-Error 'Build failed.'; exit 1 }
}
if (-not (Test-Path $exe)) { Write-Error "Could not locate $exe after build."; exit 1 }

# Sanity-check the native libraries so the user gets a clear message before the window opens.
$physx = Join-Path $root 'native\ovphysx\ovphysx\lib\ovphysx.dll'
$ovrtx = Join-Path $root 'native\ovrtx\bin\ovrtx-dynamic.dll'
if (-not (Test-Path $physx)) { Write-Warning "ovphysx.dll not found at $physx - set OVPHYSX_LIB or place it there." }
if (-not (Test-Path $ovrtx)) { Write-Warning "ovrtx-dynamic.dll not found at $ovrtx - set GEMELLI_OVRTX_DIR or place it there." }

Write-Host "Launching $exe" -ForegroundColor Green
Start-Process -FilePath $exe
