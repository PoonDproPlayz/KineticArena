---
title: Import a Fusion 360 Robot into KineticArena
description: 'Export a Fusion 360 combat robot assembly with its STL collision meshes, body tree, mass, inertia, materials, and textures for KineticArena.'
---

# Import a Fusion 360 robot into KineticArena

KineticArena imports a Fusion bundle: a manifest plus the mesh and texture files
exported beside it. Keep the complete output folder together when you copy or
move a bundle.

## Export from Fusion

1. Open the robot design in Fusion 360.
2. Open **Utilities > Scripts and Add-ins > Scripts** and add the extracted
   `Fusion360Exporter` folder as a script location. The separate exporter ZIP
   is available on the [GitHub Releases page](https://github.com/PoonDproPlayz/KineticArena/releases).
3. Run **CombatSimExport** and choose an output folder.
4. Wait for the exporter to finish. The bundle includes `manifest.json`,
   per-solid-body physics STL files, render OBJ files, and supported appearance
   textures. The manifest stores the part/occurrence tree, transforms, Fusion
   material and density, mass, and available center-of-mass and inertia data.

If the exporter reports missing external components, open the design in Fusion
and resolve or suppress broken references, then export again. The exporter may
skip unavailable occurrences and write an `export_warnings.txt` file.

## Load the bundle

1. Open KineticArena and choose **Configure bot**.
2. Select **Player bot** or **Opponent bot**, then choose **Browse...** and
   select that bot's `manifest.json`.
3. Wait for the part tree and 3D preview to load. Keep the `Parts/` and
   `Textures/` directories in their exported locations next to the manifest.
4. Assign bodies and folders to the main frame, wheel assemblies, weapon,
   forks, or excluded parts. See [Parts and mechanisms](parts-and-mechanisms.md).
5. Choose **Apply and preview** to compile the MuJoCo model. Choose **Save
   setup** to save assignments and settings beside the manifest.

Player and opponent bundles have separate manifests and setup files. The setup
file is named `combatsim_setup.json` beside its manifest. It saves assembly assignments,
axis pivots and directions, motor data, mass choices, per-part material and
friction overrides, print-profile applications, collision options, and preview
visibility. It does not replace or modify the original CAD design.

## Mesh and appearance notes

- The STL is used for physics collision and picking. Each body gets a convex
  collision hull; holes and deep recesses fill in that hull, so keep separate
  CAD bodies for surfaces that need independent contact. The OBJ carries render
  normals and UVs for smoother appearance and texture mapping.
- Fusion color, normal, and roughness maps are copied when the exporter can
  resolve them. Face-level appearances within one body are not all preserved.
- Older STL-only bundles with Fusion-native transforms can still load, but do
  not include UV texture maps. Manifests without `positionFusionMeters` and
  `rotationFusionWxyz` must be re-exported with the current Fusion exporter.
- Assignments outside the selected assembly are supported; a body does not have
  to be nested beneath its wheel or weapon folder in the Fusion tree.
- A flexible wire or cable can remain visible and contribute mass while its
  collision option is disabled. Fine STL details can create unwanted contacts,
  so disable collision for flexible or non-structural parts.

The robot mass readout combines the imported CAD bodies and configured
mechanisms. If the same physical wheel or weapon is present both in the CAD
assembly and as a stock mechanism, exclude the duplicate CAD bodies before
applying the model.

## Import a custom arena

1. Build the arena as separate solid bodies in Fusion: a floor slab, individual
   wall panels, and any other physical surfaces. Set the design origin at the
   center of the floor and place the floor top at **Z = 0**.
2. Run **CombatSimExport** for that design and keep its complete output folder.
3. Open **Settings > Arena settings**, choose **Browse arena CAD...**, select the
   exported `manifest.json`, and load it. Every exported solid becomes a static
   arena surface. The CAD bounds set the arena dimensions.
4. Set player and opponent X/Y spawn centers and heading offsets under **Bot
   starting positions**. Positions are meters from the arena origin. Enter
   centers inside the walkable area; the simulator clamps them to the arena
   bounds and each robot's footprint. These values are saved with the other
   application settings.

Arena mesh collisions use one convex hull per Fusion body, so model each wall
and the floor as its own body. The imported arena replaces the built-in floor,
walls, and roof; include a roof body if the custom arena should have one. The
exporter checks a custom Fusion physical material property whose name contains
“friction” first. Fusion's standard
physical properties do not include a friction coefficient, so otherwise the
exporter estimates one from the material name (for example plywood/laminate
0.75 and polycarbonate/acrylic 0.40). The arena settings show whether each
value came from a custom Fusion property or a material-name estimate. The
arena and robot surface estimates combine by geometric mean for each contact.

## Weapon collision proxies

MuJoCo uses a convex hull for each STL body. To approximate a concave weapon,
add several simple convex solid bodies to the Fusion design and name each one
with the prefix `CollisionProxy_` (for example `CollisionProxy_BladeA`). Export
them with the rest of the robot, then assign each proxy to the same rigid
assembly as the weapon in KineticArena. Proxy bodies contribute no mass and do
not appear in the fight render. Disable collision on the detailed weapon bodies
that the proxies replace; otherwise both shapes collide. Use enough proxies to
follow the blade or drum outline without creating many tiny collision pieces.
