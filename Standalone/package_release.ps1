param(
    [Parameter(Mandatory = $true)]
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]*$')]
    [string]$Version,
    [string]$BuildDirectory,
    [string]$OutputDirectory,
    [string]$MujocoRoot
)

$ErrorActionPreference = 'Stop'
if (-not $BuildDirectory) { $BuildDirectory = Join-Path $PSScriptRoot 'build' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $PSScriptRoot 'release-output' }
if (-not $MujocoRoot) { $MujocoRoot = Join-Path $PSScriptRoot '.deps' }

$buildOutput = Join-Path $BuildDirectory 'Release'
$exeSource = Join-Path $buildOutput 'CombatSimStandalone.exe'
$sdlSource = Join-Path $buildOutput 'SDL3.dll'
$mujocoDllSource = Join-Path $buildOutput 'mujoco.dll'
$modelSource = Join-Path $PSScriptRoot 'models\CombatRobot500g.xml'
$mujocoLicense = Join-Path $PSScriptRoot 'licenses\MuJoCo-3.3.7-LICENSE.txt'
$mujocoNotices = Join-Path $MujocoRoot 'THIRD_PARTY_NOTICES.txt'
$depsSource = Join-Path $BuildDirectory '_deps'

foreach ($requiredFile in @($exeSource, $sdlSource, $mujocoDllSource, $modelSource,
        $mujocoLicense, $mujocoNotices,
        (Join-Path $depsSource 'sdl3-src\LICENSE.txt'),
        (Join-Path $depsSource 'imgui-src\LICENSE.txt'),
        (Join-Path $depsSource 'json-src\LICENSE.MIT'))) {
    if (-not (Test-Path -LiteralPath $requiredFile -PathType Leaf)) {
        throw "Required release file is missing: $requiredFile"
    }
}

$programFilesX86 = [Environment]::GetFolderPath('ProgramFilesX86')
$vswhere = Join-Path $programFilesX86 'Microsoft Visual Studio\Installer\vswhere.exe'
if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
    throw 'Visual Studio vswhere.exe was not found; the C++ runtime files cannot be packaged.'
}
$vsInstall = & $vswhere -latest -products '*' `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
if ($LASTEXITCODE -ne 0 -or -not $vsInstall) {
    throw 'A Visual Studio C++ tool installation was not found.'
}

$redistRoot = Join-Path $vsInstall 'VC\Redist\MSVC'
$redistVersion = Get-ChildItem -LiteralPath $redistRoot -Directory -ErrorAction Stop |
    Where-Object { $_.Name -match '^\d+\.\d+\.\d+$' } |
    Sort-Object { [version]$_.Name } -Descending |
    Select-Object -First 1
if (-not $redistVersion) {
    throw "The Visual C++ redistributable files were not found under $redistRoot."
}
$crtDirectory = Join-Path $redistVersion.FullName 'x64\Microsoft.VC143.CRT'
$crtFiles = @(Get-ChildItem -LiteralPath $crtDirectory -Filter '*.dll' -File -ErrorAction Stop)
if ($crtFiles.Count -eq 0) {
    throw "The Visual C++ runtime DLLs were not found under $crtDirectory."
}

$outputRoot = [IO.Path]::GetFullPath($OutputDirectory)
New-Item -ItemType Directory -Path $outputRoot -Force | Out-Null
$packageName = "CombatSim-$Version-windows-x64"
$zipPath = Join-Path $outputRoot "$packageName.zip"
$hashPath = Join-Path $outputRoot "$packageName.sha256.txt"
$stageRoot = Join-Path $outputRoot ".staging-$([guid]::NewGuid().ToString('N'))"
$packageRoot = Join-Path $stageRoot $packageName

try {
    New-Item -ItemType Directory -Path (Join-Path $packageRoot 'models') -Force | Out-Null
    $licenseDirectory = Join-Path $packageRoot 'licenses'
    New-Item -ItemType Directory -Path $licenseDirectory -Force | Out-Null

    Copy-Item -LiteralPath $exeSource -Destination (Join-Path $packageRoot 'CombatSim.exe')
    Copy-Item -LiteralPath $sdlSource -Destination $packageRoot
    Copy-Item -LiteralPath $mujocoDllSource -Destination $packageRoot
    Copy-Item -LiteralPath $crtFiles.FullName -Destination $packageRoot
    Copy-Item -LiteralPath $modelSource -Destination (Join-Path $packageRoot 'models\CombatRobot500g.xml')

    Copy-Item -LiteralPath $mujocoLicense -Destination (Join-Path $licenseDirectory 'MuJoCo-3.3.7-LICENSE.txt')
    Copy-Item -LiteralPath $mujocoNotices -Destination (Join-Path $licenseDirectory 'MuJoCo-THIRD-PARTY-NOTICES.txt')
    Copy-Item -LiteralPath (Join-Path $depsSource 'sdl3-src\LICENSE.txt') -Destination (Join-Path $licenseDirectory 'SDL3-LICENSE.txt')
    Copy-Item -LiteralPath (Join-Path $depsSource 'imgui-src\LICENSE.txt') -Destination (Join-Path $licenseDirectory 'Dear-ImGui-LICENSE.txt')
    Copy-Item -LiteralPath (Join-Path $depsSource 'json-src\LICENSE.MIT') -Destination (Join-Path $licenseDirectory 'nlohmann-json-LICENSE.txt')

    @"
CombatSim $Version for Windows x64

To launch:
1. Extract the whole ZIP into a normal folder.
2. Double-click CombatSim.exe.

Keep the DLL files, models folder, and licenses folder next to CombatSim.exe.
The ZIP includes the stock practice scene and the runtime libraries needed to
launch it. No source checkout, setup script, batch file, compiler, or CMake is
needed to run this release.

For source builds and development instructions, visit:
https://github.com/PoonDproPlayz/KineticArena
"@ | Set-Content -LiteralPath (Join-Path $packageRoot 'README.txt') -Encoding UTF8

    $packageExe = Join-Path $packageRoot 'CombatSim.exe'
    Push-Location $env:TEMP
    try {
        & $packageExe --test-straight-drive
        if ($LASTEXITCODE -ne 0) {
            throw "The packaged application smoke test failed with exit code $LASTEXITCODE."
        }
    }
    finally {
        Pop-Location
    }

    if (Test-Path -LiteralPath $zipPath -PathType Leaf) {
        Remove-Item -LiteralPath $zipPath -Force
    }
    if (Test-Path -LiteralPath $hashPath -PathType Leaf) {
        Remove-Item -LiteralPath $hashPath -Force
    }
    Compress-Archive -Path (Join-Path $packageRoot '*') `
        -DestinationPath $zipPath -CompressionLevel Optimal
    $hash = (Get-FileHash -LiteralPath $zipPath -Algorithm SHA256).Hash.ToLowerInvariant()
    "$hash  $([IO.Path]::GetFileName($zipPath))" |
        Set-Content -LiteralPath $hashPath -Encoding ASCII

    Write-Host "Release ZIP: $zipPath"
    Write-Host "SHA-256: $hash"
}
finally {
    $outputPrefix = $outputRoot.TrimEnd([IO.Path]::DirectorySeparatorChar) + [IO.Path]::DirectorySeparatorChar
    $stageFullPath = [IO.Path]::GetFullPath($stageRoot)
    if ((Test-Path -LiteralPath $stageFullPath -PathType Container) -and
        $stageFullPath.StartsWith($outputPrefix, [StringComparison]::OrdinalIgnoreCase)) {
        Remove-Item -LiteralPath $stageFullPath -Recurse -Force
    }
}
