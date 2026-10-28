# Install CombatSim

The current app is a native Windows x64 program. The first setup builds it from
source and downloads its required libraries; a ready-to-run single-file release
is not available yet.

## First setup

1. Install or update **App Installer** so the `winget` command is available.
2. Open PowerShell in the CombatSim repository folder.
3. Run:

   ```powershell
   powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1
   ```

The script checks for Git, CMake, and Visual Studio 2022 C++ Build Tools. It
installs missing tools with WinGet, downloads MuJoCo, configures the C++ build,
and launches CombatSim. Windows may ask for permission to install the C++ build
tools. The first build also downloads SDL and Dear ImGui through CMake, so an
internet connection is required.

To install/build without opening the simulator, add `-NoRun`:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1 -NoRun
```

After a successful build, double-click `Run CombatSim.bat` in the repository
folder. It rebuilds changed source and launches the classic renderer.

## Optional PBR renderer

The Filament renderer is experimental and built separately. From the repository
folder, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\build_filament.ps1
```

This downloads and builds the Filament-enabled MuJoCo version. The initial
build is large. After it succeeds, use **Home → Settings → Graphics settings**
to switch renderers. The setting is saved for the current Windows user.

Filament provides real-time physically based raster rendering. It is not a ray
tracer. The classic renderer is the default and remains available separately.

## What gets installed

The scripts keep downloaded libraries and build output under `Standalone/`.
They do not install a packaged CombatSim application. To use a different
computer, build there from the repository or wait for a portable release.

For manual CMake instructions, see [Build from source](development/building.md).
