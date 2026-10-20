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

The 0.4 ms model timestep is independent of display frame rate. W/S drive,
A/D turn, Space spins the weapon; a gamepad uses the left stick and left
shoulder or south button. The GUI provides pause, reset and drive settings.

## Import a Fusion bot

1. In Fusion 360, rerun `Fusion360Exporter/CombatSimExport.py` on the design.
   It writes `manifest.json` and individual binary STLs in `Parts/` folders
   that follow Fusion's occurrence tree. It exports per-body mass, density,
   physical material name and available material properties, plus appearance
   color and roughness where Fusion exposes them. A bundle from the old exporter
   still loads, though it lacks the new tree and appearance fields.
2. Open the standalone app and choose **Browse...** in **CAD setup**.
   Select the bundle's `manifest.json`. Its parts tree, axis controls, and 3D
   preview are in one window. The live arena fills the main background. Use
   **Hide CAD setup** in the CombatSim controls to clear the setup overlays.
3. Expand the Fusion assembly tree. Assign individual bodies or a whole folder
   to **Main frame**, a
   wheel assembly, the weapon assembly, or **Exclude**. For example, put the
   drum, blades, and screws in one weapon assembly. Each wheel can have its own
   assembly and axis. Add more rigid assemblies if needed.
4. Click a part in the 3D preview, then use **Assign selected part** to put it
   in a rigid assembly. **Show selected**, **Hide selected**, **Show only
   selected**, and the per-part and per-folder controls change preview visibility
   without changing the physical assembly. **Deselect part** clears the
   selection. To place an axis through a polygonal hole, use **Axis target
   (weapon / wheel)** above the 3D preview to choose the exact weapon or wheel,
   press **Select hole for chosen axis**, hover a closed rim until its complete
   edge loop highlights, then click once. The tool follows connected STL
   boundary or sharp crease edges and fits a circle in the selected plane,
   putting the rotation center at the circle center and
   the axis normal to that plane. **Cancel axis** is available.
   The center and direction can also be edited as XYZ values.
   Left drag in the preview orbits and tilts; right drag pans; scroll zooms.
   The preview remains in place while dragging. Yellow lines show weapon axes;
   cyan lines show wheel axes.
5. Choose **Apply and preview**, inspect the compiled MuJoCo model, then
   **Resume** to drive. **Save setup** writes `combatsim_setup.json` beside the
   Fusion manifest; the part grouping, pivots, axes, mass overrides, colors,
   friction edits, and preview visibility load again next time. Every body has a **Weight (g)**
   field; enter a manual weight there or press **Use Fusion** to restore its
   exported value.

The app writes `combatsim_generated.xml` beside the manifest when previewing.
Included bodies require a positive mass. MuJoCo rigid-body inertia is currently
estimated from each imported part's bounding box. Fusion's physical material
data is kept for review; Fusion does not supply a universal contact friction
value, so the friction estimate is editable. Complex appearance textures are
not yet imported, and this build still uses the classic renderer.

To validate an import without opening a window:

```powershell
.\Standalone\build\Release\CombatSimStandalone.exe --validate-bundle C:\path\to\bundle\manifest.json
```
