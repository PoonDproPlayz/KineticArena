# Build from source

## Automated Windows build

From the repository root, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1 -NoRun
```

The script installs missing Git, CMake, and Visual Studio 2022 C++ Build Tools
with WinGet, downloads MuJoCo 3.3.7, configures CMake, and builds the classic
renderer. Remove `-NoRun` to launch the stock scene after building.

## Manual CMake build

Install Visual Studio 2022 with the C++ desktop workload, CMake, and Git.
Download and unpack the official MuJoCo 3.3.7 Windows x64 release. Set
`MUJOCO_ROOT` to the folder containing `include/mujoco/mujoco.h`,
`lib/mujoco.lib`, and `bin/mujoco.dll`:

```powershell
cmake -S Standalone -B Standalone/build `
  -DMUJOCO_ROOT=C:/path/to/mujoco-3.3.7-windows-x86_64
cmake --build Standalone/build --config Release
& .\Standalone\build\Release\CombatSimStandalone.exe `
  .\Standalone\models\CombatRobot500g.xml
```

CMake fetches SDL3, Dear ImGui, and nlohmann/json during configuration. An
internet connection is required the first time these dependencies are fetched.

## Filament build

`Standalone/build_filament.ps1` builds the separate experimental Filament
renderer with its compatible MuJoCo version. The classic and Filament build
directories remain separate. See [Install CombatSim](../getting-started.md) for
how to switch renderers in the app.

## Documentation preview

Install the documentation dependency and serve the pages locally:

```powershell
python -m pip install -r requirements-docs.txt
python -m mkdocs serve
```

GitHub Actions builds the site on documentation pull requests and publishes it
when changes reach `main`. See [Code layout](code-layout.md).

To enable the site in the repository, open **Settings → Pages** on GitHub and
set **Build and deployment → Source** to **GitHub Actions**. After a push to
`main`, the workflow builds only the `docs/` content and publishes it at the
project Pages URL. Pages is a public website even when the source repository is
private; GitHub's plan must allow Pages for a private repository.
