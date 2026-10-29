# Install CombatSim

CombatSim runs on Windows x64. For most users, download the
[latest release](https://github.com/PoonDproPlayz/KineticArena/releases/latest).
To edit or build the source code, follow the source setup below.

## Download and launch

1. Open the [latest release](https://github.com/PoonDproPlayz/KineticArena/releases/latest).
2. Download `CombatSim-vX.Y.Z-windows-x64.zip` and extract the whole ZIP to a
   normal folder.
3. Double-click `CombatSim.exe` in the extracted folder.

Keep the DLL files, `models` folder, and `licenses` folder beside the EXE. The
release ZIP includes the stock scene and required runtime libraries. You do not
need the source repository, CMake, Visual Studio, setup script, or batch file to
run it.

## Build from source

Use this if you want to edit the code or run the current unreleased main branch.
The one-time setup needs WinGet and an internet connection. From the repository
folder, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1 -NoRun
```

The script checks for Git, CMake, and Visual Studio 2022 C++ Build Tools. It
installs missing tools with WinGet, downloads MuJoCo, configures the C++ build,
and compiles CombatSim. Windows may ask for permission to install the C++ build
tools. The first build also downloads SDL and Dear ImGui through CMake.

After setup, launch or rebuild the current source by double-clicking
`Run CombatSim.bat` in the repository folder. In PowerShell, run it with:

```powershell
& '.\Run CombatSim.bat'
```

The batch file builds changed source and then launches the classic renderer; it
does not update your checkout. Run `git pull` first when you want the latest
main branch. The optional `-NoRun` switch on `setup.ps1` only skips the initial
launch after setup.

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

For manual CMake instructions, see [Build from source](development/building.md).
