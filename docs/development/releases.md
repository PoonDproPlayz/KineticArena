# Release packaging

CombatSim does not have a packaged GitHub release yet. The current Windows
workflow builds from source and needs CMake, a C++ toolchain, MuJoCo, SDL, and
other build dependencies.

## Planned one-file download

The requested user experience is a ZIP containing a single `CombatSim.exe`.
The practical way to provide that is a self-extracting portable launcher:

1. Build a release version of the app and collect the required runtime DLLs,
   Filament assets (for the PBR build), and stock model files.
2. Bundle and compress those files as resources inside the launcher executable.
3. On first run, extract them to a versioned directory under the user's local
   application data, then start CombatSim. Do not require administrator access.
4. Include third-party notices and licenses with the release information.
5. Produce a ZIP containing only the launcher, then attach it to a tagged GitHub
   Release. Verify it on a clean Windows machine that has no developer tools.

That packaging flow is a design target only. It has not been implemented, and
there is no current release ZIP to download. A normal C++ executable cannot
launch the complete app from one file unless its runtime libraries and data are
also embedded or extracted at run time.
