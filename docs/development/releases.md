---
title: KineticArena Windows Releases and Fusion 360 Exporter
description: 'Download the KineticArena Windows simulator release or separate Fusion 360 exporter package, and learn how release builds are created.'
---

# KineticArena releases and packaging

Windows builds are published on the
[GitHub Releases page](https://github.com/PoonDproPlayz/KineticArena/releases).
Each release includes two separate ZIP downloads:

- `CombatSim-<version>-windows-x64.zip` contains `CombatSim.exe`, its runtime
  DLLs, the stock practice model, and third-party license notices. Extract the
  whole ZIP and double-click the EXE. Keep the supporting files next to it.
- `CombatSim-<version>-fusion360-exporter.zip` contains the Fusion 360 exporter
  script, Fusion configuration, and installation instructions. Download it
  separately when you want to export a robot from Fusion 360.

Each ZIP has its own `.sha256.txt` checksum asset.

The simulator ZIP uses the classic renderer and contains SDL3, MuJoCo, and the
Visual C++ runtime files required on a clean Windows x64 machine. It has one
application executable; the DLLs and model data beside it are needed to run.

## Automated release builds

The `.github/workflows/release.yml` workflow builds the Windows x64 Release
configuration, packages and smoke-tests it, then creates a GitHub Release with
both ZIPs and their SHA-256 checksums attached. Start it either by pushing a
`v*` tag or from the Actions tab.

To start it from GitHub, open **Actions > Build and publish Windows release >
Run workflow**, select the `main` branch, enter a version such as `v0.1.0`, and
run the workflow. It builds that version from the current `main` branch and
creates its release tag automatically.

For a new version from Git, commit and push the desired source to `main`, then
create and push a version tag. For example:

```powershell
git switch main
git pull
git tag v0.1.0
git push origin v0.1.0
```

Watch the **Build and publish Windows release** workflow in the repository's
Actions tab. The release is published after the build and packaged-app smoke
test pass.

## Local package check

After building from source, create the same ZIP locally with:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\package_release.ps1 -Version v0.1.0
```

Both ZIPs and their checksums are written to the ignored
`Standalone/release-output/` directory. This command packages files locally;
the GitHub Actions workflow publishes the release assets.
