# CombatSim standalone prototype

This native driving app loads the stock MJCF or a Fusion bot bundle directly. It
uses MuJoCo 3.3.7's classic renderer, SDL3 for input/windowing, and Dear ImGui
for the bot editor. Filament requires upgrading MuJoCo; that upgrade follows
validation of this native import and physics path.

## Build on Windows

From the repository root, run this single command in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1
```

The script installs missing Git, CMake, and Visual Studio 2022 C++ Build Tools
with WinGet, downloads MuJoCo 3.3.7, builds, then launches the app. It may
request Windows administrator approval for the C++ tools. It is safe to rerun
after an interrupted install. To build without launching, add `-NoRun`.

After setup, double-click `Run CombatSim.bat` in the project root whenever you
want to start the app. It launches the existing build without installing or
downloading anything.

### Manual build

Install Visual Studio 2022 with C++ desktop tools, CMake, and Git. Download and
unpack the official MuJoCo 3.3.7 Windows release (headers, import library and
DLL are required). From the repository root:

```powershell
cmake -S Standalone -B Standalone/build -DMUJOCO_ROOT=C:/path/to/mujoco-3.3.7-windows-x86_64
cmake --build Standalone/build --config Release
./Standalone/build/Release/CombatSimStandalone.exe ./Assets/CombatSim/Models/CombatRobot500g.xml
```

CMake fetches SDL3 and Dear ImGui at configure time. `MUJOCO_ROOT` must contain
`include/mujoco/mujoco.h`, `lib/mujoco.lib` and `bin/mujoco.dll`.

The 0.4 ms model timestep is independent of display frame rate. The app opens
to a Home screen with **Practice drive** and **Configure bot** entry points; a
persistent top navigation bar switches between Home, Drive, and Bot workshop.
W/S drive, A/D turn, and Space spins the weapon; a gamepad uses the left stick
and left shoulder or south button. The Drive screen provides pause, reset, an
editable wheel target from 0 to 5,000 RPM, weapon target and spin-up settings,
and live telemetry showing actual weapon RPM plus each wheel's actual RPM and
left/right side averages.

The arena stays at 2,500 Hz. Its transparent side walls now use a 10 cm thick
collision volume, a 1 cm early contact margin, and a stiffer contact response;
the inner wall faces remain at the same 2 m arena boundary. This reduces high
speed penetration without spending four times more simulation work per second.

### Bot-specific motor torque

In **Bot workshop**, open **Drive motor settings** and assign wheel assemblies
to a motor group. Wheels driven by the same physical motor use the same group;
the group's torque is divided evenly across those wheel assemblies. Enable the
wheel torque model for the group, then enter the total number of motors in that
group and each motor's Kv
(RPM/V), and peak current per motor (A), plus the robot battery pack's nominal
voltage. Those are the core inputs. Kv gives the speed/torque constant, so the
simulator estimates motor torque from current and converts it through the
configured gear ratio. Phase-to-phase resistance and an ESC phase-current cap
are optional; leave either at 0 if unknown. The model then uses a current-
limited linear motor curve. Peak current is treated as torque-producing motor
current. The simulator does not model winding temperature or reduce torque
after a peak-current burst, so it can sustain the peak torque indefinitely. A
catalog value that means battery input current is only an approximation.

The weapon remains on the RPM controller for now. Weapon motor torque and
spin-up based on its assembly moment of inertia are future work.

Shared motor groups share torque but do not yet impose chain, belt, differential,
or other rigid wheel-speed coupling. Each wheel joint remains independently
simulated.

The default 1:1 gear ratio and 100% efficiency are explicit assumptions. If a
motor has a gearbox, enter motor RPM divided by output RPM as its ratio; this
changes both output speed and torque. Efficiency can remain at 100% until a
better value is available. A LiPo pack's nominal voltage can be estimated as
3.7 V times its cell count. These settings are saved in
`combatsim_setup.json` with the bot. When the model is disabled, the assembly
continues to use the legacy prescribed-speed actuator.

## Import a Fusion bot

1. In Fusion 360, rerun `Fusion360Exporter/CombatSimExport.py` on the design.
   It writes `manifest.json` and individual binary STLs in `Parts/` folders
   that follow Fusion's occurrence tree. It exports per-body mass, density,
   physical material name and available material properties, plus appearance
   color and roughness where Fusion exposes them. A bundle from the old exporter
   still loads, though it lacks the new tree and appearance fields.
2. Open the standalone app and choose **Configure bot** (or **Bot workshop** in
   the top navigation). Choose **Browse...** and select the bundle's
   `manifest.json`. The workshop presents the parts tree and configuration
   controls beside the selectable 3D preview.
3. Expand the Fusion assembly tree. Assign individual bodies or a whole folder
   to **Main frame**, a
   wheel assembly, the weapon assembly, or **Exclude**. For example, put the
   drum, blades, and screws in one weapon assembly. Give each physical wheel
   its own assembly, including its tire, hub, motor, and fasteners; keep front
   and rear wheels separate because each assembly has one pivot and hinge.
    Use **Gravity-driven fork modules** to create the number of separate fork
    hinges you need, then assign each module its own CAD bodies. Fork hinges
    have no motor; gravity moves them. Set each fork's pivot, axis, and travel
    range (default +/-90 degrees from the imported CAD pose). Each module has
    a **Collide with main frame** checkbox. Explicit MuJoCo contact pairs are
    generated only when that module's checkbox is enabled. Add more rigid
    assemblies if needed.
4. Click a part in the 3D preview, then use **Assign selected part** to put it
   in a rigid assembly. **Show selected**, **Hide selected**, **Show only
   selected**, and the per-part and per-folder controls change preview visibility
   without changing the physical assembly. **Deselect part** clears the
   selection. To place an axis through a polygonal hole, use **Axis target
    (weapon / wheel / fork)** above the 3D preview to choose the exact moving part,
   press **Select hole for chosen axis**, hover a closed rim until its complete
   edge loop highlights, then click once. The tool follows connected STL
   boundary and crease edges across chamfers, then fits a circle in the selected plane,
   putting the rotation center at the circle center and
   the axis normal to that plane. **Cancel axis** is available.
    The center and direction can also be edited as XYZ values. Fork travel
    limits are editable in degrees relative to the imported pose.
   Left drag in the preview orbits and tilts; right drag pans; scroll zooms.
   The preview remains in place while dragging. Yellow lines show weapon axes;
    cyan lines show wheel axes; purple lines show fork hinges.
5. Choose **Apply and preview** to compile the MuJoCo model. Choose **Drive**
   in the top navigation to enter the arena, then use the drive panel to pause
   or reset. **Save setup** writes `combatsim_setup.json` beside the
   Fusion manifest; the part grouping, pivots, axes, mass overrides, colors,
   friction edits, and preview visibility load again next time. Every body has a **Weight (g)**
   field; enter a manual weight there or press **Use Fusion** to restore its
   exported value. The main window reports left and right wheel-joint counts;
   turning in place requires at least one drive assembly on each side.
   Wheel-axis signs are normalized so a fitted hole normal pointing either
   direction still produces the expected forward/reverse motion.

The app writes `combatsim_generated.xml` beside the manifest when previewing.
Included bodies require a positive mass. Rotating assemblies combine each
assigned body's Fusion mass and transformed center of mass into one
mass-weighted assembly COM. When available, the bundle also carries each
body's centroidal inertia tensor from Fusion. The importer rotates those
tensors into the assembly frame, scales them when a manual mass override is
used (assuming the body's mass distribution remains similar), and combines
them about the group COM with the parallel-axis theorem. Model added material
as a separate body when its location within a part matters. MuJoCo receives
the resulting full inertia tensor. Older bundles and bodies without Fusion
inertia keep the bounding-box estimate. The **Combined COM
offset from shaft** readout shows the radial offset that drives vibration at
speed. Set the shaft center to the measured, physically balanced shaft
location; changing the axis changes the simulated imbalance. Fusion's physical
material data is kept for review; Fusion does not supply a universal contact
friction value, so the friction estimate is editable. Complex appearance
textures are not yet imported, and this build still uses the classic renderer.

To validate an import without opening a window:

```powershell
.\Standalone\build\Release\CombatSimStandalone.exe --validate-bundle C:\path\to\bundle\manifest.json
```
