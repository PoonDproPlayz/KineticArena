# Release packaging

Windows builds are published on the
[GitHub Releases page](https://github.com/PoonDproPlayz/KineticArena/releases).
Each release ZIP contains one `CombatSim.exe` to launch, its runtime DLLs, the
stock practice model, and third-party license notices. Extract the whole ZIP
and double-click the EXE. Keep the supporting files next to it.

The release uses the classic renderer. The ZIP also contains SDL3, MuJoCo, and
the Visual C++ runtime files required on a clean Windows x64 machine. The
application executable is the only EXE in the package; the DLLs and model data
are needed for it to run.

## Automated release builds

Pushing a `v*` tag to GitHub starts
`.github/workflows/release.yml`. The workflow builds the Windows x64 Release
configuration, packages and smoke-tests it, then creates a GitHub Release with
the ZIP and SHA-256 checksum attached.

For a new version, commit and push the desired source to `main`, then create and
push a version tag. For example:

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

The output and checksum are written to the ignored
`Standalone/release-output/` directory. This command packages files locally;
the GitHub Actions workflow publishes the release assets.
