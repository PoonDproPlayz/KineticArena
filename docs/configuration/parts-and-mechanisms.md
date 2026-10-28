# Parts and mechanisms

The Fusion assembly tree and preview work together. Hover over a preview part
to highlight it. Click a part to select it, focus its tree entry, expand its
configuration, and scroll it into view. Folder assignments can include several
CAD bodies at once.

## Assign bodies

Use the per-part or per-folder assignment controls to choose:

- **Main frame** for the chassis and fixed structural parts.
- A separate **wheel assembly** for each physical wheel. Keep front and rear
  assemblies separate so each can have its own pivot, axis, and axle position.
- The **weapon assembly** for the spinning body and attached teeth or fasteners.
- A **gravity-driven fork module** for each independently hinged fork.
- **Exclude** for visual or CAD bodies that should not enter the simulation.

Additional rigid assemblies can hold extra mechanisms. An unknown material can
still use a manual part mass. The app distributes a configured assembly mass
across its selected bodies using CAD masses when available.

## Set a wheel or weapon axis

1. Choose the moving assembly in **Axis target (weapon / wheel / fork)**.
2. Choose **Select hole for chosen axis**.
3. Hover over a round hole rim until the complete edge loop highlights, then
   click it. Chamfered rims are supported.
4. Review the fitted center and axis direction. You can edit the coordinates
   directly or flip the axis direction.

Axis selection can use visible CAD bodies outside the moving assembly. The
selected axis target determines which assembly receives the fitted axis. Axis
centers are shown in millimeters; saved setup files retain the existing meter
units internally.

Use the preview camera controls to inspect the fit: left-drag orbits, right-drag
pans, and the scroll wheel zooms. The axis marker checkbox controls whether
wheel and weapon axes are shown in the preview and during a drive session.

## Gravity-driven forks

Set **Gravity-driven fork modules** to the number of independent forks, then
assign each module its own CAD bodies. Each fork uses a passive hinge with an
editable pivot, axis, and travel range. Use **Collide with main frame** when
the fork should physically contact the robot chassis. Fixed forks should remain
assigned to the frame instead of a hinge module.

Forks, fixed frame parts, and the chassis can make floor contact. If a fork is
intended to scrape the floor, keep its collision enabled and check that the
imported shape and placement match the real bot. Disable collisions for flexible
or nonphysical visual parts that create unwanted contacts.

## Mass and material

Each body can use its Fusion mass, a manual mass, or an applied printed-part
estimate. Material presets provide starting density and friction values. The
friction estimate is editable because real contact depends on both surfaces and
their finish; it is not a universal coefficient for a material name.

Presets cover UHMW, 7075 aluminum, tool steels, PA12 and generic nylon, ABS,
TPU 95A, platinum-cure silicone 40A and 20A, carbon-fiber composite, and
unknown/custom materials. Density and friction estimates remain editable for
each part.

The combined center of mass and inertia affect motion and weapon spin-up. For a
rotating assembly, the importer combines body mass properties around the
selected axis. Set the axis to the manufactured shaft center to represent the
real balance. A small offset can produce vibration; an incorrect axis can make
a balanced assembly appear unbalanced.

Use **Show only selected** and the body/folder visibility controls to inspect
the CAD. Preview visibility does not change simulation assignments.
