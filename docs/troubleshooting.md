# Troubleshooting

## CombatSim does not launch

- If the executable is missing, run `Standalone/setup.ps1` from PowerShell and
  wait for the build to finish.
- If a Windows DLL error appears, rerun the setup script so CMake can copy the
  runtime dependencies beside the executable.
- If the Filament option is selected but its build is missing, build it with
  `Standalone/build_filament.ps1` or switch back to the classic renderer.
- The app currently builds from source; downloading the repository does not
  provide a ready-to-run release executable.

## Fusion bundle says a mesh is missing

Keep the whole exported folder together. `manifest.json` refers to mesh files
in the bundle's `Parts/` folders and textures in `Textures/`; do not move only
the manifest. Re-export from Fusion if any referenced file is missing. If the
exporter writes `export_warnings.txt`, check those occurrence paths in Fusion
for broken external references.

## Parts appear missing or incorrectly grouped

Confirm that the correct `manifest.json` is loaded and that the mesh files are
present. In the bot workshop, expand the assembly tree and check the part or
folder assignment and preview visibility. A part assigned to **Exclude** is
not added to the simulation. Preview visibility controls rendering and does
not alter the assignment.

## A round hole will not select for an axis

Choose the moving **Axis target** first, then enable hole selection. Hover over
the full circular rim until its edge loop highlights before clicking. You can
select a reference hole on another visible body; the axis target determines
which assembly is changed. If the imported surface is open or lacks a complete
rim, enter the axis center and direction directly.

## Bot shakes or has the wrong mass

Check that the imported body mass and material are correct, that duplicate CAD
wheel/weapon bodies are not included alongside stock mechanisms, and that the
axis passes through the manufactured shaft center. For printed parts, apply a
print profile and check the estimated mass and center of mass. Mass and balance
estimates depend on the source Fusion data and the configured axis.

## One PC cannot join its own hosted match

Run two app instances, host in the first, and enter `127.0.0.1`, the same port,
and the same session key in the second. For two computers, use the host's LAN
IPv4 address and allow CombatSim through the private-network firewall.

## Match result or damage features are missing

The current build does not yet include a match timer, winner declaration, or
per-part damage model. These are planned features, not settings hidden in the
bot workshop.
