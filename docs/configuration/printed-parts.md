# Estimate printed-part mass

Printed-part estimates are configured per body in the bot workshop. They use
the body's Fusion-exported density, not a slicer-profile material density. The
profile describes how the part is printed; it does not replace the material
assigned in Fusion.

## Create and apply profiles

1. Open the part's **Printed part** settings.
2. Create or edit a reusable print profile with layer height, line width, wall
   loops, top and bottom layers, sparse infill density, and infill pattern.
3. Choose the face that sits on the print bed to set the build direction.
4. Apply the profile to the selected part.
5. Review the estimated mass and center of mass, then save the bot setup.

Profiles live in a separate profile library so you can switch print settings
without overwriting the settings already applied to each body. Applying a
profile stores a snapshot on that part, including its bed direction. Editing a
library profile later does not silently change the part; apply it again when
you want the new settings.

## How to read the estimate

CombatSim estimates shells, top/bottom regions, and sparse infill from the
exported mesh and print settings. The density comes from Fusion's physical
material data, with Fusion mass and volume used to recover it when possible. If
the bundle has no usable Fusion density, the app reports that it cannot estimate
printed mass rather than substituting the print-profile name as a material.

This is a geometric approximation, not a PrusaSlicer, OrcaSlicer, or Cura tool
path simulation. It does not reproduce every slicer's line placement, seam,
support, bridge, or pattern-generation behavior. The printed mass/COM/inertia
estimate affects the physics model, while the CAD surface remains the collision
shape. Use it to compare designs and profiles, then compare the result against
a real printed part's measured mass.
