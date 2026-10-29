# Import a Fusion 360 robot

CombatSim imports a Fusion bundle: a manifest plus the mesh and texture files
exported beside it. Keep the complete output folder together when you copy or
move a bundle.

## Export from Fusion

1. Open the robot design in Fusion 360.
2. Open **Utilities → Add-ins → Scripts and Add-ins** and add the
   `Fusion360Exporter` folder from this repository as a script location.
3. Run **CombatSimExport** and choose an output folder.
4. Wait for the exporter to finish. The bundle includes `manifest.json`,
   per-solid-body physics STL files, render OBJ files, and supported appearance
   textures. The manifest stores the part/occurrence tree, transforms, Fusion
   material and density, mass, and available center-of-mass and inertia data.

If the exporter reports missing external components, open the design in Fusion
and resolve or suppress broken references, then export again. The exporter may
skip unavailable occurrences and write an `export_warnings.txt` file.

## Load the bundle

1. Open CombatSim and choose **Configure bot**.
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

- The STL is used for picking and physics bounds. The OBJ carries render normals
  and UVs for smoother appearance and texture mapping.
- Fusion color, normal, and roughness maps are copied when the exporter can
  resolve them. Face-level appearances within one body are not all preserved.
- Older STL-only bundles with Fusion-native transforms can still load, but do
  not include UV texture maps. Manifests without `positionFusionMeters` and
  `rotationFusionWxyz` must be re-exported with the current Fusion exporter.
- Assignments outside the selected assembly are supported; a body does not have
  to be nested beneath its wheel or weapon folder in the Fusion tree.
- A flexible wire or cable can remain visible and contribute mass while its
  collision option is disabled. A rigid convex collision shape may snag the
  floor if used for a flexible part.

The robot mass readout combines the imported CAD bodies and configured
mechanisms. If the same physical wheel or weapon is present both in the CAD
assembly and as a stock mechanism, exclude the duplicate CAD bodies before
applying the model.
