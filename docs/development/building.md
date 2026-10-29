# Build from source

For a ready-to-run application, use the
[latest Windows release](https://github.com/PoonDproPlayz/KineticArena/releases/latest).
Follow this page when you want to build or modify the main branch.

## Automated Windows build

For the first setup, open PowerShell at the repository root and run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1 -NoRun
```

The script installs missing Git, CMake, and Visual Studio 2022 C++ Build Tools
with WinGet, downloads MuJoCo 3.3.7, configures CMake, and builds the classic
renderer. `-NoRun` skips the initial launch. After setup, double-click
`Run CombatSim.bat` in the repository root to build changed source and launch
the simulator. The same command from PowerShell is:

```powershell
& '.\Run CombatSim.bat'
```

The batch file does not update the checkout; run `git pull` before it if you
want the latest main branch.

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

The guide is a MkDocs Material site. From the repository root, install its
requirements once and start the local preview server:

```powershell
python -m pip install -r requirements-docs.txt
python -m mkdocs serve
```

Then open http://127.0.0.1:8000 in a browser. MkDocs rebuilds the preview as
you edit documentation; press Ctrl+C in PowerShell to stop it.

To generate a static copy, run `python -m mkdocs build --strict`. The files are
written to `site/`.

## Package a release ZIP locally

After the automated source build succeeds, package the executable and its
runtime files with:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\package_release.ps1 -Version v0.1.0
```

The ZIP and SHA-256 checksum are written to `Standalone/release-output/`. The
package smoke test runs before the ZIP is created. To publish a GitHub Release,
push a version tag such as `v0.1.0`; the release workflow builds and uploads the
ZIP automatically. See [Release packaging](releases.md).

## Public documentation site

The guide is published at <https://poondproplayz.github.io/KineticArena/>. After
GitHub Pages is enabled, pushes to `main` that change the docs, MkDocs config,
documentation requirements, or deployment workflow build and deploy the site
automatically. Pull requests run the strict build check without deploying.

To enable publishing, open the repository on GitHub and go to **Settings >
Pages > Build and deployment > Source**, select **GitHub Actions**, and save.
Then push this workflow and the site files to `main`. Check the **Actions** tab
for the first **Publish documentation** run; its deployment job publishes the
site. See [Code layout](code-layout.md).
