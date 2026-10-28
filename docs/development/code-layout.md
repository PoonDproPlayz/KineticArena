# Code layout

The native app is separated into a few main modules:

| File | Responsibility |
|---|---|
| `Standalone/main.cpp` | App startup, command-line diagnostics, simulation loop, input, rendering, and top-level pages |
| `Standalone/AppSettings.h/.cpp` | Per-user arena, spawn, lighting, renderer, and preset preferences |
| `Standalone/FusionBundle.h/.cpp` | Fusion manifest and mesh loading, bot setup, mass properties, MuJoCo model generation, and bot workshop UI |
| `Standalone/LanSession.h/.cpp` | LAN transport, encrypted package exchange, input messages, and physics snapshots |
| `Standalone/models/CombatRobot500g.xml` | Default practice arena and stock robot mechanisms |
| `Fusion360Exporter/CombatSimExport.py` | Fusion 360 bundle exporter |
| `docs/` | User and developer documentation, published through GitHub Pages |

The normal runtime flow starts by resolving a model path, loading application
preferences and bot setup, and compiling a MuJoCo model. The main loop then
updates input and motor commands, advances MuJoCo, collects telemetry, renders
the arena and CAD preview, and draws the selected UI page. Fusion bot packages
can be compiled from files or kept in memory for a LAN session.

Application preferences are separate from per-bot setup. Renderer choice,
arena size, spawn layout, lighting, and named arena presets are saved by SDL in
the current user's preferences. Wheel assignments, axes, masses, motor data,
print settings, and part visibility are saved beside the Fusion manifest.

## Editing guidance

- Keep simulation data conversions explicit. CAD/setup coordinates are stored
  in meters even when the UI displays millimeters.
- Add new app-wide settings to `AppSettings`; keep bot-specific configuration
  in `FusionBundle`.
- Keep optional renderers in separate build configurations so renderer-specific
  libraries do not leak into the default classic build.
- Preserve older setup-file keys when changing serialization, or add a clear
  migration path before removing them.
