<#
.SYNOPSIS
    Builds the bundled Python for the mass-properties service.

.DESCRIPTION
    Fills UAVSim/Assets/MassService~/python/ with the Windows embeddable
    CPython 3.13.14 plus the pinned packages in mass-service-requirements.txt,
    then runs a smoke test (100 mm cube + 5 point masses) against the bundled
    udp_listener.py under the embedded interpreter.

    The service scripts themselves live in UAVSim/Assets/MassService~/mass-properties/
    and are committed; python/ is a build output and is git-ignored.

    Windows only: python.org publishes the embeddable package for Windows only.

.PARAMETER BuildPython
    Any Python with pip, used only to run `pip install --target`. It is not
    modified. pip is told the target platform explicitly (win_amd64, CPython
    3.13), so the build interpreter's own version does not matter.

.PARAMETER Clean
    Delete an existing python/ folder first. Without it, the script refuses
    to overwrite.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tools\build-mass-service.ps1 -Clean
#>
param(
    [string]$BuildPython = 'C:\Users\Owner\Documents\MassCompose\.venv\Scripts\python.exe',
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'

$PythonVersion = '3.13.14'
$EmbedUrl      = "https://www.python.org/ftp/python/$PythonVersion/python-$PythonVersion-embed-amd64.zip"

$RepoRoot     = Split-Path $PSScriptRoot -Parent
$ServiceRoot  = Join-Path $RepoRoot 'UAVSim\Assets\MassService~'
$PythonDir    = Join-Path $ServiceRoot 'python'
$ScriptsDir   = Join-Path $ServiceRoot 'mass-properties'
$SitePackages = Join-Path $PythonDir 'Lib\site-packages'
$Requirements = Join-Path $PSScriptRoot 'mass-service-requirements.txt'
$PythonExe    = Join-Path $PythonDir 'python.exe'

function Step($message) { Write-Host "==> $message" -ForegroundColor Cyan }

if (-not (Test-Path (Join-Path $ScriptsDir 'udp_listener.py')) -or
    -not (Test-Path (Join-Path $ScriptsDir 'mass_composer.py'))) {
    throw "Service scripts missing from $ScriptsDir"
}
if (-not (Test-Path $BuildPython)) {
    throw "Build Python not found at '$BuildPython'. Pass -BuildPython <path to any python.exe with pip>."
}
if (Test-Path $PythonDir) {
    if (-not $Clean) { throw "$PythonDir already exists. Re-run with -Clean to rebuild it." }
    Step "Removing existing $PythonDir"
    Remove-Item -Recurse -Force $PythonDir
}

# 1. Embeddable CPython. Downloaded into its own empty temp folder.
$TempDir = Join-Path ([IO.Path]::GetTempPath()) ("mass-service-build-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $TempDir | Out-Null
try {
    $Zip = Join-Path $TempDir "python-$PythonVersion-embed-amd64.zip"
    Step "Downloading $EmbedUrl"
    [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
    $ProgressPreference = 'SilentlyContinue'   # PS 5.1 progress bar makes Invoke-WebRequest very slow
    Invoke-WebRequest -Uri $EmbedUrl -OutFile $Zip -UseBasicParsing

    Step "Unpacking into $PythonDir"
    Expand-Archive -Path $Zip -DestinationPath $PythonDir

    # 2. Packages. Binary wheels only, for the embedded interpreter's platform.
    Step "Installing pinned packages into $SitePackages"
    & $BuildPython -m pip install `
        --disable-pip-version-check --no-warn-script-location `
        --target $SitePackages `
        --platform win_amd64 --implementation cp --python-version 3.13 `
        --only-binary=:all: `
        -r $Requirements
    if ($LASTEXITCODE -ne 0) { throw "pip install failed (exit $LASTEXITCODE)" }

    # 3. Path file. REQUIRED: with a ._pth present the script's own folder is
    #    not on sys.path, so without the last line `import mass_composer` fails.
    Step "Writing python313._pth"
    $PthLines = @('python313.zip', '.', 'Lib\site-packages', '..\mass-properties')
    [IO.File]::WriteAllLines((Join-Path $PythonDir 'python313._pth'), $PthLines, (New-Object Text.ASCIIEncoding))

    # 4. Pre-compile so the first launch does not compile numpy/trimesh.
    Step "Pre-compiling site-packages"
    & $PythonExe -m compileall -q $SitePackages | Out-Null
    if ($LASTEXITCODE -ne 0) { throw "compileall failed (exit $LASTEXITCODE)" }

    # 5. Smoke test under the embedded interpreter: 100 mm cube at 1000 kg/m^3,
    #    battery 0.5 kg at the origin, four 0.05 kg motors at (+-0.15, +-0.15, 0).
    #    Hand arithmetic: Ixx = Iyy = 1*0.1^2/6 + 4*0.05*0.15^2 = 0.0061667,
    #    Izz = 1*0.1^2/6 + 4*0.05*(2*0.15^2) = 0.0106667, armLength = 0.21213.
    Step "Smoke test"
    $SmokeScript = Join-Path $TempDir 'smoke_test.py'
    $StlPath     = Join-Path $TempDir 'cube_100mm.stl'
    @'
import math, struct, sys
import trimesh
import udp_listener

stl_path = sys.argv[1]
trimesh.creation.box(extents=[100.0, 100.0, 100.0]).export(stl_path)

nodes = [(1, 0.0, 0.0, 0.0, 0.5)] + [(0, sx * 0.15, sy * 0.15, 0.0, 0.05)
                                    for sx in (1, -1) for sy in (1, -1)]
path_bytes = stl_path.encode("utf-8")
req = struct.pack("<BfI", 40, 1000.0, len(path_bytes)) + path_bytes + struct.pack("<I", len(nodes))
for n in nodes:
    req += struct.pack("<Bffff", *n)

resp = udp_listener.handle_request(req)
if resp[0] != 41:
    msg_len = struct.unpack_from("<I", resp, 1)[0]
    sys.exit("FAIL: error reply: " + resp[5:5 + msg_len].decode("utf-8", "replace"))
if len(resp) != 45:
    sys.exit(f"FAIL: expected 45 bytes, got {len(resp)}")

v = struct.unpack("<11f", resp[1:])
com, moments, arm = v[0:3], sorted(v[3:6]), v[10]
expected = sorted([0.0061667, 0.0061667, 0.0106667])
ok = (all(abs(c) < 1e-6 for c in com)
      and all(math.isclose(a, b, rel_tol=1e-4) for a, b in zip(moments, expected))
      and math.isclose(arm, 0.21213, rel_tol=1e-4))
print(f"python {sys.version.split()[0]}  com={com}  moments={moments}  arm={arm:.5f}")
sys.exit(0 if ok else "FAIL: values do not match hand arithmetic")
'@ | Set-Content -Path $SmokeScript -Encoding ASCII

    & $PythonExe $SmokeScript $StlPath
    if ($LASTEXITCODE -ne 0) { throw "Smoke test failed (exit $LASTEXITCODE)" }
}
finally {
    Remove-Item -Recurse -Force $TempDir -ErrorAction SilentlyContinue
}

$SizeMB = [Math]::Round(((Get-ChildItem $ServiceRoot -Recurse -File | Measure-Object Length -Sum).Sum / 1MB), 1)
Step "Done. $ServiceRoot is $SizeMB MB."
