# Arena and graphics settings

Open **Settings** from Home. Arena settings and graphics settings are separate
pages.

## Arena

- Choose a square arena side length from **0.75 m to 10 m**. The default is
  **2 m**.
- Choose **Center line** starts to place the robots at opposite ends facing one
  another, or **Opposite diagonal** to place them at opposite corners.
- Choose a built-in Small, Standard, or Large size preset, or save a custom
  arena size under a name.
- The visible walls are clear acrylic panels and the roof closes the arena.
  The floor is rendered as opaque warm-white laminate.

Changing arena size pauses and resets the bots so the floor, walls, and spawn
positions update together. A 1 cm clearance is used when placing bot geometry
near a wall.

## Graphics

Adjust ambient, key, fill, and camera lighting, or select the classic renderer
or experimental Filament PBR renderer. The settings are saved for the current
Windows user.

Filament must be built once before switching to it. See [Install CombatSim](../getting-started.md#optional-pbr-renderer). The graphics option restarts into the
matching executable. The Filament renderer uses real-time rasterization, not
ray tracing.

The simulator uses an Earth gravity magnitude of **9.80665 m/s²** and a **0.4
ms (2,500 Hz)** physics timestep. Display frame rate is separate from the
physics update rate.
