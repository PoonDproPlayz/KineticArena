# CombatSim project handoff

Updated: 2026-10-29. Keep this file focused on the application, implementation,
and validation. Do not put private robot names, exported CAD paths, or actual
hardware ratings from personal builds here.

## Project

CombatSim is a native Windows desktop practice simulator for small combat
robots. MuJoCo handles physics; SDL3 and Dear ImGui provide the application
window and interface. The supported application is in Standalone. The stock
practice model is Standalone/models/CombatRobot500g.xml.

## Source map

- Standalone/main.cpp contains the app shell, Home and Drive screens, controller
  input and mixing, arena settings, telemetry, and automated test entry points.
- Standalone/FusionBundle.cpp and FusionBundle.h load Fusion bundles, edit body
  assignments, mass and print profiles, axis settings, and generate MuJoCo
  models.
- Fusion360Exporter/CombatSimExport.py is the Fusion 360 add-in.
- Standalone/LanSession.cpp and LanSession.h handle direct LAN sessions.
- docs/ contains the MkDocs Material user and developer guides.
- .github/workflows/release.yml builds Windows ZIP assets on v* tags.
- Standalone/package_release.ps1 packages and smoke-tests releases;
  Standalone/publish_release.ps1 creates the release and uploads assets.

## Fusion workflow

Export a design with the Fusion360Exporter add-in. The output is a manifest,
one physics STL per body, and optional render OBJ and texture assets. The
manifest preserves the occurrence tree, Fusion-native transforms, available
material and appearance data, body mass and density, center of mass, and
inertia when Fusion provides it.

In Configure, assign bodies to the frame, wheels, weapon, forks, or Exclude.
Moving assemblies share a selected rotation axis. Each body can use Fusion mass,
a manual weight, or an applied printed-part profile. Setup and profile choices
are saved with the bundle or in the user's application settings.

Current manifests must include positionFusionMeters and rotationFusionWxyz.
Re-export older manifests with the current add-in if those fields are missing.
Tests should use temporary bundle copies when model generation writes beside a
manifest. Preserve external CAD bundles and saved user setups.

## Driving and physics

The default build uses MuJoCo 3.3.7 and a 0.0004 second simulation step. It
supports keyboard driving, USB controller mixing, opponent control, motor
models, wheel slip telemetry, configurable arenas, and LAN play.

Motor behavior is a model estimate. It uses configured motor and drivetrain
values but does not yet model battery sag, motor heating, or rigid chain/belt
wheel-speed coupling. Real-hardware calibration is still needed. The controller
setup now guides device connection and per-input detection; each model's input
mapping and mix are saved under its CAD bundle or XML path, while the selected
physical controller remains a computer-wide preference. Physical USB controller
behavior and the visible CAD axis highlight still need manual confirmation on
the user's computer.

## Documentation site

The guide is a MkDocs Material site published with GitHub Pages at
https://poondproplayz.github.io/KineticArena/. Pushes to `main` that change the
documentation, MkDocs config, requirements, or deployment workflow build and
deploy automatically. Pull requests run a strict build check without
deploying. The repository's Pages source must be set to **GitHub Actions** in
Settings > Pages > Build and deployment. The Pages site is public; repository
visibility and the plan's support for Pages on private repositories determine
whether the source can stay private.

For local preview, install its requirements once from the repository root,
then run:

- python -m pip install -r requirements-docs.txt
- python -m mkdocs serve

Open http://127.0.0.1:8000 in a browser. Press Ctrl+C to stop the preview.
Run python -m mkdocs build --strict to create a static copy in site/.

## Windows releases

Tag pushes matching v* or a manual run from the Actions tab on `main` can start
the Windows release workflow. It builds the classic renderer, packages
CombatSim.exe with SDL3, MuJoCo, Visual C++ runtime DLLs, the
stock model XML, and third-party notices, then runs a packaged-app drive smoke
test and uploads a ZIP plus SHA-256 file to GitHub Releases. The ZIP has one
application EXE to launch; required runtime files stay beside it. Create local
packages after building with `powershell -ExecutionPolicy Bypass -File
Standalone/package_release.ps1 -Version v0.1.0`.

## Build, launch, and validation

From the repository root:

- First-time setup and build: powershell -ExecutionPolicy Bypass -File
  .\Standalone\setup.ps1
- Build without launching: add -NoRun to that command.
- Everyday launch: Run CombatSim.bat.
- Build a release ZIP locally: powershell -ExecutionPolicy Bypass -File
  .\Standalone\package_release.ps1 -Version v0.1.0.
- Release build: cmake --build Standalone/build --config Release
- Fusion exporter tests: python -B -m unittest
  Standalone.tests.test_fusion_exporter
- Full generated-bundle smoke suite: python -B
  Standalone/tests/smoke_bundle.py
  Standalone/build/Release/CombatSimStandalone.exe

The Release build, 14 Fusion exporter tests, full bundle smoke suite, and
git diff --check passed after the native-transform cleanup on 2026-10-29.
