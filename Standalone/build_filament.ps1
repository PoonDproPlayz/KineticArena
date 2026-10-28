param([switch]$NoRun)

$ErrorActionPreference = 'Stop'
$standaloneDir = $PSScriptRoot
$projectDir = Split-Path $standaloneDir -Parent
$depsDir = Join-Path $standaloneDir '.deps'
$sourceDir = Join-Path $depsDir 'mujoco-filament-src'
$buildDir = Join-Path $standaloneDir 'filament-build'
$fetchDir = Join-Path $env:TEMP 'CombatSimFilamentDeps'
$sourceCmake = Join-Path $sourceDir 'CMakeLists.txt'
$filamentCmake = Join-Path $sourceDir 'src\experimental\filament\CMakeLists.txt'

try {
    if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
        throw 'Git is required to fetch MuJoCo 3.4 and its Filament renderer.'
    }
    if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
        throw 'CMake is required to build the Filament test version.'
    }
    $vswhere = Join-Path ([Environment]::GetFolderPath('ProgramFilesX86')) 'Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path -LiteralPath $vswhere)) {
        throw 'Visual Studio 2022 C++ Build Tools are required to compile Filament.'
    }
    $vs = & $vswhere -latest -products '*' -version '[17.0,18.0)' `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
    if ($LASTEXITCODE -ne 0 -or -not $vs) {
        throw 'Visual Studio 2022 C++ Build Tools are required to compile Filament.'
    }

    New-Item -ItemType Directory -Path $depsDir -Force | Out-Null
    if (-not (Test-Path -LiteralPath $sourceCmake)) {
        Write-Host 'Fetching MuJoCo 3.4.0 source for the optional Filament renderer...'
        & git clone --depth 1 --branch 3.4.0 https://github.com/google-deepmind/mujoco.git $sourceDir
        if ($LASTEXITCODE -ne 0) { throw 'Could not fetch MuJoCo 3.4.0.' }
    }

    # MuJoCo's Filament build uses Dear ImGui core but not its SDL2 backend.
    # The upstream dependency CMake still expects that unused target to exist.
    $filamentText = [System.IO.File]::ReadAllText($filamentCmake)
    $workaround = 'if(NOT TARGET SDL2::SDL2-static)'
    if (-not $filamentText.Contains($workaround)) {
        $needle = 'include\(third_party_deps/filament\)\r?\n'
        $insert = @'
if(NOT TARGET SDL2::SDL2-static)
    add_library(SDL2::SDL2-static INTERFACE IMPORTED GLOBAL)
endif()
if(TARGET dear_imgui_SDL2)
    set_target_properties(dear_imgui_SDL2 PROPERTIES EXCLUDE_FROM_ALL TRUE)
endif()
'@
        $updated = [regex]::Replace($filamentText, $needle, { param($match) $match.Value + $insert }, 1)
        if ($updated -eq $filamentText) {
            throw 'MuJoCo 3.4 Filament CMake files did not match the expected layout.'
        }
        [System.IO.File]::WriteAllText($filamentCmake, $updated)
    }
    $projectText = [System.IO.File]::ReadAllText($sourceCmake)
    $hookMarker = 'COMBATSIM_SOURCE_ROOT}/Standalone/FilamentBuildHook.cmake'
    if (-not $projectText.Contains($hookMarker)) {
        $hook = @'

if(DEFINED COMBATSIM_SOURCE_ROOT)
  include("${COMBATSIM_SOURCE_ROOT}/Standalone/FilamentBuildHook.cmake")
endif()
'@
        [System.IO.File]::AppendAllText($sourceCmake, $hook)
    }
    $pbrSource = Join-Path $sourceDir 'src\experimental\filament\assets\pbr.mat'
    Copy-Item -LiteralPath (Join-Path $standaloneDir 'filament\pbr.mat') `
        -Destination $pbrSource -Force

    Write-Host 'Configuring the Filament build (first build compiles Filament and may take several minutes)...'
    $configureArgs = @(
        '-S', $sourceDir, '-B', $buildDir, '-G', 'Visual Studio 17 2022', '-A', 'x64',
        "-DFETCHCONTENT_BASE_DIR=$fetchDir",
        '-DFETCHCONTENT_FULLY_DISCONNECTED=OFF',
        '-DCMAKE_PROJECT_mujoco_INCLUDE=',
        "-DCOMBATSIM_SOURCE_ROOT=$projectDir",
        '-DCOMBATSIM_USE_FILAMENT=ON',
        '-DMUJOCO_USE_FILAMENT=ON',
        '-DMUJOCO_USE_FILAMENT_VULKAN=OFF',
        '-DMUJOCO_BUILD_EXAMPLES=OFF',
        '-DMUJOCO_BUILD_SIMULATE=OFF',
        '-DMUJOCO_BUILD_STUDIO=OFF',
        '-DMUJOCO_BUILD_TESTS=OFF',
        '-DMUJOCO_BUILD_TESTS_WASM=OFF'
    )
    $sdl3Source = Join-Path $projectDir 'Standalone\build\_deps\sdl3-src'
    $imguiSource = Join-Path $projectDir 'Standalone\build\_deps\imgui-src'
    $jsonSource = Join-Path $projectDir 'Standalone\build\_deps\json-src'
    if (Test-Path -LiteralPath $sdl3Source) { $configureArgs += "-DFETCHCONTENT_SOURCE_DIR_SDL3=$sdl3Source" }
    if (Test-Path -LiteralPath $imguiSource) { $configureArgs += "-DFETCHCONTENT_SOURCE_DIR_IMGUI=$imguiSource" }
    if (Test-Path -LiteralPath $jsonSource) { $configureArgs += "-DFETCHCONTENT_SOURCE_DIR_JSON=$jsonSource" }
    & cmake @configureArgs
    if ($LASTEXITCODE -ne 0) { throw "Filament CMake configure failed ($LASTEXITCODE)." }

    Write-Host 'Building CombatSim with the PBR renderer...'
    $msbuild = Join-Path ($vs | Select-Object -First 1) 'MSBuild\Current\Bin\amd64\MSBuild.exe'
    $appProject = Join-Path $buildDir 'combat-sim-standalone\CombatSimStandalone.vcxproj'
    $exe = Join-Path $buildDir 'bin\Release\CombatSimStandalone.exe'
    $filamentLibrary = Join-Path $buildDir 'lib\Release\mujoco_filament.lib'
    $compiledMaterial = Join-Path $buildDir 'src\experimental\filament\assets\pbr.filamat'
    $fullBuildRequired = -not (Test-Path -LiteralPath $exe) -or
        -not (Test-Path -LiteralPath $filamentLibrary) -or
        -not (Test-Path -LiteralPath $compiledMaterial)
    if (-not $fullBuildRequired) {
        $fullBuildRequired = (Get-Item (Join-Path $standaloneDir 'FilamentMultiContext.cpp')).LastWriteTimeUtc -gt
            (Get-Item $filamentLibrary).LastWriteTimeUtc -or
            (Get-Item (Join-Path $standaloneDir 'FilamentBuildHook.cmake')).LastWriteTimeUtc -gt
            (Get-Item $filamentLibrary).LastWriteTimeUtc -or
            (Get-Item (Join-Path $standaloneDir 'filament\pbr.mat')).LastWriteTimeUtc -gt
            (Get-Item $compiledMaterial).LastWriteTimeUtc
    }
    $buildArgs = @($appProject, '/p:Configuration=Release', '/p:Platform=x64', '/m:1', '/v:minimal')
    if (-not $fullBuildRequired) { $buildArgs += '/p:BuildProjectReferences=false' }
    & $msbuild @buildArgs
    if ($LASTEXITCODE -ne 0) { throw "Filament build failed ($LASTEXITCODE)." }

    if (-not (Test-Path -LiteralPath $exe)) {
        $exe = Join-Path $buildDir 'bin\CombatSimStandalone.exe'
    }
    if (-not (Test-Path -LiteralPath $exe)) {
        throw 'Build finished, but the CombatSim Filament executable was not found in the MuJoCo bin output folder.'
    }
    Write-Host "Built: $exe"
    if (-not $NoRun) {
        $model = Join-Path $standaloneDir 'models\CombatRobot500g.xml'
        Write-Host 'Launching the Filament build with CombatRobot500g.xml...'
        & $exe $model
        if ($LASTEXITCODE -ne 0) { throw "CombatSim Filament exited with code $LASTEXITCODE." }
    }
} catch {
    Write-Error $_
    exit 1
}
