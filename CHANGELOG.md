# Changelog

Notable changes to KineticArena releases are listed here. Release-specific
notes are also included in the GitHub Release description.

## [v1.1.0] - 2026-11-09

Compared with `v0.1.0`:

### Driving and fight review

- Expanded USB controller setup with per-bot saved maps, input detection,
  weights, offsets, expo, deadzones, and live response curves.
- Added self-contained `.kar` fight replays that include the model and control
  trace, with 3D playback, camera controls, playback speed, and input
  re-simulation.
- Added simulation-speed and physics-step timing telemetry.

### Robot setup and physics

- Expanded wheel and weapon motor models with KV or rated RPM, current limits,
  gearing, battery voltage sag estimates, and weapon spin-up time estimates.
- Added configurable arena dimensions, saved size presets, custom Fusion arena
  import, material-based surface-friction estimates, and editable spawn poses.
- Added convex collision-hull preview and support for multiple lightweight
  collision proxy bodies to approximate complex weapon shapes.
- Added optional impact-force failure thresholds for wheel and weapon assemblies.
  They default to off and disable the assigned drive actuator; this release does
  not simulate part fracture or progressive structural damage.
- Moved model generation and compilation off the main UI path to keep the
  interface responsive during model changes.

### Importer, downloads, and docs

- Expanded Fusion export/import support for body hierarchy, material properties,
  appearance data, and separate collision proxy bodies.
- Added a separate Fusion 360 exporter ZIP to releases, with SHA-256 checksums
  for both the simulator and exporter downloads.
- Added an in-app one-time Cloudflare signaling-service deployment launcher.
  Internet matches are not enabled yet; the game transport remains LAN-only.
- Updated the KineticArena documentation, search metadata, and Google indexing
  files.

## [v0.1.0]

- Initial Windows simulator release with the MuJoCo-based arena and Fusion 360
  robot import workflow.
