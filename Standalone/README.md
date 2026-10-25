# CombatSim standalone prototype

This native driving app loads the stock MJCF or a Fusion bot bundle directly. It
uses MuJoCo 3.3.7's classic renderer by default, SDL3 for input/windowing, and
Dear ImGui for the bot editor. An optional experimental Filament build uses
MuJoCo 3.4.0 for PBR rendering; the classic build remains the default.

## Build on Windows

From the repository root, run this single command in PowerShell:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1
```

The script installs missing Git, CMake, and Visual Studio 2022 C++ Build Tools
with WinGet, downloads MuJoCo 3.3.7, builds, then launches the app. It may
request Windows administrator approval for the C++ tools. It is safe to rerun
after an interrupted install. To build without launching, add `-NoRun`.

### Experimental PBR renderer

To build the optional Filament renderer and launch the same robot in it, run:

```powershell
powershell -ExecutionPolicy Bypass -File .\Standalone\build_filament.ps1
```

This fetches MuJoCo 3.4.0 and Filament and makes a separate build. The first
build is large and can take several minutes. It does not replace the default
3.3.7 build. Filament adds physically based materials, image-based reflections,
and better light response; it is a real-time raster renderer rather than a
ray-tracing renderer. Fusion roughness is used when supplied, and material
names provide editable-style estimates for metallic surfaces and plastics.
The simulator still uses the model's 0.4 ms (2,500 Hz) physics timestep and standard Earth gravity (`9.80665 m/s²` downward).
Use `-NoRun` to build without opening the test scene.

Home now has a **Settings** entry with separate **Arena settings** and
**Graphics settings** pages. Graphics settings includes a Filament PBR checkbox.
Switching renderers saves the selection and restarts with the matching build;
build Filament once with the command above before selecting it. The selection
persists when starting from `Run CombatSim.bat` or either renderer executable.

To check the renderer and capture a frame from both the drive scene and the
separate CAD-preview context, run:

```powershell
Standalone\filament-build\bin\Release\CombatSimStandalone.exe `
  --test-filament-render Assets\CombatSim\Models\CombatRobot500g.xml `
  "$env:TEMP\CombatSimFilamentPreview.bmp"
```

The smoke test exits nonzero if either render is blank or it cannot write the
bitmap.

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
Drive, and Bot workshop. **Settings** on Home separates **Arena settings** from
**Graphics settings**. Arena settings sets the square arena side length in
meters (0.75-10 m, default 2 m) and selects either a face-to-face center-line
spawn or opposite-diagonal spawn. Graphics settings adjusts ambient, key, fill,
and camera lighting and toggles between classic and Filament PBR rendering.
Choose the built-in Small, Standard, or Large preset, or save the current size
under a custom preset name for later use. Custom
presets and arena settings save per user. Resizing updates the current floor,
clear side panels, hidden collision walls, and spawn positions, then pauses and
resets the bots. Drive also has a **Controller mixer** button.
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

The arena stays at 2,500 Hz. Its square side length is adjustable in meters in
Arena settings; 2 m is the default. Its visible side skins are 8 mm clear acrylic;
separate invisible collision volumes extend 60 cm outside each inner wall face
to catch fast impacts without making the panels look like solid blue blocks.
The roof has the same hidden catch volume. A 1 cm spawn margin keeps each
robot's measured geometry clear of the walls, and Reset reapplies the selected
center-line or diagonal layout. The floor uses a matte warm-white laminate
material. Lighting defaults are subdued and migrate older brighter preferences
once; the sliders remain available in Arena settings.

Configure shows rotating-assembly axis centers in millimeters. Setup files
continue to store `pivotMeters`, so existing saved bot setups remain compatible.

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

Each bot's workshop also has a **Bot rotation about Z** setting. It rotates the
assembled bot as one root transform; part coordinates, assignments, and fitted
CAD axes stay in the original CAD frame. The arena floor is fully opaque in
both the stock and Fusion-generated models.

## Import a Fusion bot

1. In Fusion 360, rerun `Fusion360Exporter/CombatSimExport.py` on the design.
   It writes `manifest.json`, a binary STL for selection/physics bounds, and a
   render OBJ for each solid body in `Parts/` folders that follow Fusion's
   occurrence tree. The OBJ keeps shared vertex normals and UV coordinates so
   lighting stays smooth across tessellation triangles on a CAD surface.
   Supported Fusion color, normal, and roughness image maps are copied into a
   portable `Textures/` folder and assigned to the render material. Appearance
   maps are resolved per body/occurrence; separate face-level appearance maps
   within one body are not exported yet. It exports per-body mass, density,
   physical material name and available material properties, plus appearance
   color and roughness where Fusion exposes them. Old STL-only bundles still
   load; MuJoCo now smooths their visual normals automatically, though they do
   not contain UVs or bundled image maps. If the design has broken external
   references, the exporter skips unavailable components, completes the rest,
   and writes `export_warnings.txt` with the missing occurrence paths.
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
friction value, so the friction estimate is editable. The importer loads bundled
Fusion color maps in the classic renderer. The optional Filament renderer also
uses supported normal and roughness maps for PBR shading.

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

## Peer-to-peer LAN multiplayer

Both players should load and configure a file-backed Fusion bot first. In
CombatSim, open **LAN Multiplayer** from Home (or Drive), then:

1. The host leaves the port at `29811`, clicks **Host match**, and shares the
   displayed 32-character session key, port, and their local IPv4 address.
   `ipconfig` in PowerShell shows the IPv4 address for the active Wi-Fi or
   Ethernet adapter.
2. The other player enters that address, the same port and session key, then
   clicks **Join match**. Allow CombatSim through Windows Defender Firewall on
   private networks if prompted.
3. Both apps transfer the configured bot package, build the same MuJoCo model,
   and start after both sides report ready. Each player drives their own bot
   with W/A/S/D and spins its weapon with Space; a gamepad's left stick and
   left shoulder or south button also work. The host owns the simulation and
   sends physics snapshots to the joining PC. When testing both instances on
   one PC, only the focused window reads local keyboard and gamepad input.
   Drive shows the local player's wheel and weapon RPM telemetry. The joining
   window reads telemetry for its own robot and shows its axis markers only
   when that local bot's Configure checkbox is enabled.

The bot package is sent over an AES-GCM encrypted TCP connection and imported
from memory; the app does not extract received STL or manifest files. It clears
temporary package buffers after loading and drops the extra in-memory STL copies
once MuJoCo has compiled the model. The compiled model still contains the
geometry needed to render and simulate the bot, so a determined opponent could
inspect process memory or reconstruct geometry from the running model. This
reduces casual file copying; it cannot guarantee CAD secrecy or prevent cheats.

This first version connects directly to one reachable host. It has no internet
relay, NAT traversal, router setup, or host migration. Both PCs need a network
route between them. To test on one PC, start CombatSim twice, host in the first
window, then join from the second using `127.0.0.1`, the same port, and the
copied key. Each window should load a local bot first. The host can end the
match at any time; the app then restores the local manifest-backed bot setup.

The self-test runs two different synthetic bundles through localhost and checks
the encrypted package exchange, shared model layout, readiness handshake,
remote controls, state snapshots, wrong-key rejection, and the input timeout:

```powershell
python .\Standalone\tests\smoke_bundle.py .\Standalone\build\Release\CombatSimStandalone.exe
```

Main-frame geoms, fixed structural parts, and gravity-hinged or named fork
geoms use an overdamped MuJoCo floor-contact response (`solref="0.02 2"`).
MuJoCo applies this through explicit floor-only contact pairs, preserving the
normal response for bot-to-bot impacts. Each part keeps its collision toggle;
the editable friction estimate is retained for fork contacts and the regular
floor friction mix is retained for the chassis. Assign forks to a gravity-
driven module when the real fork can pivot; leave them on the frame when they
are fixed.
