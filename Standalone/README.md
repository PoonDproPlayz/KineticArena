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
want to start the app. It builds changed source before launch, so it does not
keep opening an older executable. It does not install or download anything.

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
to a Home screen with **Practice drive**, **Configure bot**, and **Controller
mixing** buttons; a persistent top navigation bar switches between Home,
Drive, and Bot workshop. Drive also has a **Controller mixer** button.
W/S drive and A/D turn the main bot; Space spins its weapon. I/K drive and J/L
turn the opponent; M spins its weapon. A gamepad uses the left stick and left
shoulder or south button to control the main bot. Controller mixing opens a setup window for
raw USB joysticks, up to 16 configurable axis/button input channels, and a
mix for each drive motor group and weapon. Each input mix line has editable
weight, offset, and expo; the response graph follows the selected live channel
with a moving dot while the other inputs stay at their current values. Its
default RC tank/elevon mix is left = Ch1 - Ch2, right = Ch1 + Ch2, weapon =
Ch3; axis reverse and deadzone are adjustable, and live channel meters help
identify which stick or button is which. Mixer settings save on each change
and app exit in the app's user preferences, independently of and retained
across loaded robots. The Drive screen provides
pause, reset, a
motor-derived wheel speed estimate, an optional manual wheel target from 0 to
5,000 RPM, and live telemetry showing actual weapon RPM plus each wheel's RPM
and left/right side averages. Weapon motor configuration is in Bot workshop;
when enabled, its RPM comes from the motor model and assembly inertia. Each
driven wheel also reports **GRIP**, **SLIPPING**,
or **AIRBORNE**, with measured tread speed relative to the floor at its contact
patch. Torque-mode wheel commands ramp to a new target over 0.30 seconds to
reduce abrupt launch and reversal loads; the manual wheel RPM override remains
direct. Drag the Drive sidebar's right edge to reveal more of its fixed-size
content. The Bot workshop divider does the same for the setup panel beside the
robot preview. Text wrapping and control widths stay fixed as either panel is
resized; narrow panels clip content until widened. The layout updates when the
resize handle is released.

The arena stays at 2,500 Hz. Its transparent side walls now use a 10 cm thick
collision volume, a 1 cm early contact margin, and a stiffer contact response;
the inner wall faces remain at the same 2 m arena boundary. This reduces high
speed penetration without spending four times more simulation work per second.

### Bot-specific motor torque

In the Drive tab, full throttle now uses each side's motor-derived no-load
wheel speed, calculated from Kv or the motor's RPM-at-voltage rating, the pack
voltage, and the gearbox ratio. The displayed estimate comes from the motor and
battery inputs; loaded RPM then emerges from the torque model and simulated
wheel load. If a side has no motor model, the legacy 424 RPM target is used and
the UI says motor data is missing.

For a forced-speed check, enable **Manual wheel RPM override** and enter a
target such as 4000 RPM. That uses a separate speed servo and bypasses the
drive motor torque model, so it is useful for testing but does not represent
realistic motor-limited driving.

In **Bot workshop**, open **Drive motor settings** and assign wheel assemblies
to a motor group. Wheels driven by the same physical motor use the same group;
the group's torque is divided evenly across those wheel assemblies. Enable the
wheel torque model for the group, then enter the total number of motors in that
group and each motor's peak current per motor (A), plus the robot battery
pack's nominal voltage. Choose **Brushless (BLDC)** or **Brushed DC**, then
enter either Kv (RPM/V) or a published motor speed and its test voltage. The
RPM-at-voltage option is treated as no-load speed and scaled with pack voltage;
use a no-load rating when the manufacturer provides one. Both forms derive the
same equivalent speed/torque constant for the model. Phase-to-phase resistance
for brushless motors, armature resistance for brushed motors, and the motor
controller's current limit are optional; leave unknown values at 0. Without a
resistance value, the simulator uses a current-limited approximate motor curve.
Peak current is treated as torque-producing motor current. The simulator does
not model winding temperature or reduce torque after a peak-current burst, so
it can sustain the peak torque indefinitely. A catalog value that means
battery input current is only an approximation.

The **Drive layout** selector supports 2WD and 4WD. In 2WD, choose the front or
rear axle; only that axle's left and right wheel assemblies enter MuJoCo. The
other axle's assignments and motor settings remain saved, and switching back
to 4WD restores the stored assemblies. Each wheel assembly has a **Wheel
position** field so imported or custom layouts can be assigned to the correct
axle.

The **Weapon motor settings** section configures motors for weapon assemblies.
Enter motor count, speed data (Kv or RPM at voltage), peak current, battery
voltage, and gearing. With its torque model enabled, holding the weapon control
applies motor torque; the imported assembly inertia determines acceleration
and spin-up. Releasing the control removes motor drive and the weapon coasts
under simulated mechanical losses.
With the model disabled, the existing target-RPM and spin-up controls remain
available. Bot workshop and Drive show an estimated 0-to-90% no-load spin-up
time calculated from the motor torque curve and the weapon's inertia about its
selected axis. The estimate does not include battery sag, impacts, or thermal
derating. These settings persist with the bot setup and remain enabled after
reloading it.

Shared motor groups share torque but do not yet impose chain, belt, differential,
or other rigid wheel-speed coupling. Each wheel joint remains independently
simulated.

Gear ratios use **input:output** notation, such as `16:1` or `3:2`; the
simulator uses input RPM divided by output RPM. Older numeric ratio settings
still load and are written in colon format when the setup is saved. The default `1:1`
ratio and 100% efficiency are explicit assumptions. Efficiency can remain at
100% until a better value is available. A LiPo pack's nominal voltage can be
estimated as 3.7 V times its cell count. These settings are saved in
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
   friction edits, collision toggles, and preview visibility load again next time. Flexible
   wire and cable bodies start with **Use for collisions** off because their rigid STL
   hulls can snag the floor; they remain visible and keep their mass. Every body has a **Weight (g)**
   field; enter a manual weight there or press **Use Fusion** to restore its
   exported value. The main window reports left and right wheel-joint counts;
   turning in place requires at least one drive assembly on each side.
    Wheel-axis signs are normalized so a fitted hole normal pointing either
    direction still produces the expected forward/reverse motion. CAD spawn
    heading is inferred from the assigned left/right wheel layout so the
    player faces the opponent and an imported opponent faces the player.

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

To check that the stock robot tracks straight under a forward command, run:

```powershell
.\Standalone\build\Release\CombatSimStandalone.exe --test-straight-drive
```

This runs the stock `CombatRobot500g.xml` for one simulated second with only
the robot and floor colliding, then reports lateral drift, yaw change, and
individual wheel RPM. An optional XML path can be supplied, but this check
requires the stock actuator and body names. Add `--arena` to start at the
normal spawn points and include arena and opponent collisions; this reports
the result without treating collision-driven yaw as a drivetrain test failure.

To test a saved CAD bot's forward drive using its motor, wheel, and axis setup,
run:

```powershell
.\Standalone\build\Release\CombatSimStandalone.exe --test-straight-drive-bundle C:\path\to\bundle\manifest.json
```

This generates the model beside the manifest, disables arena and opponent
collisions for the test, then reports chassis yaw, each wheel's RPM, and its
floor grip/slip status after one simulated second. It also reports chassis and
fork floor contacts during the settling and powered portions separately.

Main-frame geoms, fixed structural parts, and gravity-hinged or named fork
geoms use an overdamped MuJoCo floor-contact response (`solref="0.02 2"`).
MuJoCo applies this through explicit floor-only contact pairs, preserving the
normal response for bot-to-bot impacts. Each part keeps its collision toggle;
the editable friction estimate is retained for fork contacts and the regular
floor friction mix is retained for the chassis. Assign forks to a gravity-
driven module when the real fork can pivot; leave them on the frame when they
are fixed.
