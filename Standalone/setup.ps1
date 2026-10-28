param([switch]$NoRun)

$ErrorActionPreference = 'Stop'
$standaloneDir = $PSScriptRoot
$depsDir = Join-Path $standaloneDir '.deps'
$buildDir = Join-Path $standaloneDir 'build'
$mujocoZip = Join-Path $depsDir 'mujoco-3.3.7-windows-x86_64.zip'
$mujocoUrl = 'https://github.com/google-deepmind/mujoco/releases/download/3.3.7/mujoco-3.3.7-windows-x86_64.zip'

function Require-WinGet {
    if (-not (Get-Command winget -ErrorAction SilentlyContinue)) {
        throw 'WinGet is missing. Install or update Windows App Installer from the Microsoft Store, then rerun this script.'
    }
}

function Refresh-Path {
    $machinePath = [Environment]::GetEnvironmentVariable('Path', 'Machine')
    $userPath = [Environment]::GetEnvironmentVariable('Path', 'User')
    $env:Path = "$machinePath;$userPath;$env:Path"
}

function Install-WinGetPackage([string]$packageId) {
    Require-WinGet
    Write-Host "Installing $packageId..."
    & winget install --id $packageId --exact --source winget --accept-source-agreements --accept-package-agreements
    if ($LASTEXITCODE -ne 0) { throw "WinGet failed to install $packageId (exit code $LASTEXITCODE)." }
    Refresh-Path
}

function Find-VsCpp {
    $vswhere = Join-Path ([Environment]::GetFolderPath('ProgramFilesX86')) 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) { return $null }
    $installation = & $vswhere -latest -products '*' -version '[17.0,18.0)' -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $installation) { return $null }
    return ($installation | Select-Object -First 1)
}

try {
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) { Install-WinGetPackage 'Git.Git' }
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) { Install-WinGetPackage 'Kitware.CMake' }

    if (-not (Find-VsCpp)) {
        Require-WinGet
        Write-Host 'Installing Visual Studio 2022 C++ Build Tools. Windows may ask for administrator approval.'
        & winget install --id Microsoft.VisualStudio.2022.BuildTools --exact --source winget `
            --accept-source-agreements --accept-package-agreements `
            --override '--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended'
        if ($LASTEXITCODE -ne 0) { throw "C++ Build Tools installation failed (exit code $LASTEXITCODE)." }
        if (-not (Find-VsCpp)) {
            throw 'C++ Build Tools are still unavailable. Restart Windows if the installer requested it, then rerun this script.'
        }
    }

    New-Item -ItemType Directory -Path $depsDir -Force | Out-Null
    $mujocoRoot = $depsDir
    $header = Join-Path $mujocoRoot 'include\mujoco\mujoco.h'
    $library = Join-Path $mujocoRoot 'lib\mujoco.lib'
    $dll = Join-Path $mujocoRoot 'bin\mujoco.dll'
    if (-not ((Test-Path -LiteralPath $header) -and (Test-Path -LiteralPath $library) -and (Test-Path -LiteralPath $dll))) {
        if (-not (Test-Path -LiteralPath $mujocoZip)) {
            Write-Host 'Downloading MuJoCo 3.3.7...'
            Invoke-WebRequest -Uri $mujocoUrl -OutFile $mujocoZip -UseBasicParsing
        }
        Write-Host 'Extracting MuJoCo...'
        Expand-Archive -LiteralPath $mujocoZip -DestinationPath $depsDir -Force
        $candidate = Get-ChildItem -LiteralPath $depsDir -Filter mujoco.h -Recurse -File |
            Where-Object { $_.FullName -like '*\include\mujoco\mujoco.h' } |
            Select-Object -First 1
        if (-not $candidate) { throw 'MuJoCo archive did not contain include\mujoco\mujoco.h.' }
        $mujocoRoot = Split-Path (Split-Path (Split-Path $candidate.FullName -Parent) -Parent) -Parent
        $header = Join-Path $mujocoRoot 'include\mujoco\mujoco.h'
        $library = Join-Path $mujocoRoot 'lib\mujoco.lib'
        $dll = Join-Path $mujocoRoot 'bin\mujoco.dll'
        if (-not ((Test-Path -LiteralPath $header) -and (Test-Path -LiteralPath $library) -and (Test-Path -LiteralPath $dll))) {
            throw "MuJoCo release is incomplete at $mujocoRoot. Expected the header, import library, and DLL."
        }
    }

    Write-Host 'Configuring CombatSim...'
    & cmake -S $standaloneDir -B $buildDir -G 'Visual Studio 17 2022' -A x64 "-DMUJOCO_ROOT=$mujocoRoot"
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit code $LASTEXITCODE)." }
    Write-Host 'Building CombatSim...'
    & cmake --build $buildDir --config Release
    if ($LASTEXITCODE -ne 0) { throw "CMake build failed (exit code $LASTEXITCODE)." }

    $exe = Join-Path $buildDir 'Release\CombatSimStandalone.exe'
    if (-not (Test-Path -LiteralPath $exe)) { throw "Build finished but executable was not found at $exe." }
    Write-Host "Built: $exe"
    if (-not $NoRun) {
        $model = Join-Path $standaloneDir 'models\CombatRobot500g.xml'
        Write-Host 'Launching CombatSim...'
        & $exe $model
        if ($LASTEXITCODE -ne 0) { throw "CombatSim exited with code $LASTEXITCODE." }
    }
} catch {
    Write-Error $_
    exit 1
}
