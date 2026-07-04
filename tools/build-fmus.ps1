# Builds the ovfmi demo FMUs (FMI 2.0 co-simulation, win64) with MSVC and packages the
# conveyor_demo.ssp archive that scenes\conveyor_fmi.usda references.
#
# Sources come from the vendored ovfmi checkout (see README -> Setup):
#   external\omniverse-labs\projects\ovfmi\fmu\fmi2\*         FMU C++ + modelDescription.xml
#   external\omniverse-labs\projects\ovfmi\ssp\conveyor_demo  SSP wiring (SystemStructure.ssd)
# Output (gitignored, regenerate any time):
#   scenes\fmi\*.fmu, scenes\fmi\conveyor_demo.ssp
#
# Requires Visual Studio with the MSVC x64 C++ toolset (located via vswhere).
param()

$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$ovfmi = Join-Path $root 'external\omniverse-labs\projects\ovfmi'
$outDir = Join-Path $root 'scenes\fmi'
$buildDir = Join-Path $root 'out\fmu-build'

if (-not (Test-Path (Join-Path $ovfmi 'fmu\fmi2'))) {
    Write-Host 'ovfmi sources not found under external\omniverse-labs; clone them first (see README).' -ForegroundColor Red
    exit 1
}

# Locate the MSVC environment via vswhere -> vcvars64.bat.
$vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path $vswhere)) { Write-Host 'vswhere.exe not found; install Visual Studio with C++ tools.' -ForegroundColor Red; exit 1 }
$vsPath = & $vswhere -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if (-not $vsPath) { Write-Host 'No Visual Studio installation with the MSVC x64 toolset found.' -ForegroundColor Red; exit 1 }
$vcvars = Join-Path $vsPath 'VC\Auxiliary\Build\vcvars64.bat'

New-Item -ItemType Directory -Force $outDir | Out-Null
New-Item -ItemType Directory -Force $buildDir | Out-Null
Add-Type -AssemblyName System.IO.Compression.FileSystem

function Build-Fmu([string]$srcDir, [string]$cppName, [string]$fmuName) {
    $src = Join-Path $ovfmi "fmu\fmi2\$srcDir\$cppName"
    $desc = Join-Path $ovfmi "fmu\fmi2\$srcDir\modelDescription.xml"
    $work = Join-Path $buildDir $fmuName
    $stage = Join-Path $work 'stage'
    Remove-Item $work -Recurse -Force -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force (Join-Path $stage 'binaries\win64') | Out-Null

    $dll = Join-Path $stage "binaries\win64\$fmuName.dll"
    $obj = Join-Path $work "$fmuName.obj"
    Write-Host "  cl $cppName -> $fmuName.dll"
    # cl needs the vcvars environment; run the compile inside one cmd invocation.
    $line = '"{0}" >nul 2>nul && cl /nologo /LD /O2 /std:c++17 /EHsc /utf-8 /DFMI_VERSION=2 "/Fo{1}" "/Fe{2}" "{3}" /link /NOLOGO' -f $vcvars, $obj, $dll, $src
    # Write-Host (not pipeline output): a function's pipeline output becomes its RETURN value.
    cmd /c $line | Where-Object { $_ } | ForEach-Object { Write-Host "    $_" }
    if ($LASTEXITCODE -ne 0 -or -not (Test-Path $dll)) { Write-Host "  FAILED: $fmuName" -ForegroundColor Red; exit 1 }

    Copy-Item $desc (Join-Path $stage 'modelDescription.xml')
    $fmu = Join-Path $outDir "$fmuName.fmu"
    Remove-Item $fmu -ErrorAction SilentlyContinue
    [System.IO.Compression.ZipFile]::CreateFromDirectory($stage, $fmu)
    return $fmu
}

Write-Host 'Building FMI 2.0 FMUs (win64)...' -ForegroundColor Cyan
$presence = Build-Fmu 'presence_sensor' 'presence_sensor.cpp' 'PresenceSensor'
$controller = Build-Fmu 'conveyor_controller' 'conveyor_controller.cpp' 'ConveyorController'
$motor = Build-Fmu 'motor_drive' 'motor_drive.cpp' 'MotorDrive'

Write-Host 'Packaging conveyor_demo.ssp...' -ForegroundColor Cyan
$sspStage = Join-Path $buildDir 'ssp-stage'
Remove-Item $sspStage -Recurse -Force -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force (Join-Path $sspStage 'resources') | Out-Null
Copy-Item (Join-Path $ovfmi 'ssp\conveyor_demo\SystemStructure.ssd') $sspStage
Copy-Item $presence, $controller, $motor (Join-Path $sspStage 'resources')
$ssp = Join-Path $outDir 'conveyor_demo.ssp'
Remove-Item $ssp -ErrorAction SilentlyContinue
[System.IO.Compression.ZipFile]::CreateFromDirectory($sspStage, $ssp)

Write-Host "Done. Output in $outDir" -ForegroundColor Green
Get-ChildItem $outDir | ForEach-Object { '  {0,10:n0}  {1}' -f $_.Length, $_.Name }
