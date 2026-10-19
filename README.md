# CombatSim

A Unity starter project for a physics-based combat robot simulator.

## Requirements

- Unity Hub
- Unity 6 LTS (6000.0 or newer)
- Windows Build Support in Unity Hub if you want to build a Windows executable

Install Unity Hub from [unity.com/download](https://unity.com/download), then use
the Installs tab to install a Unity 6 LTS editor. Open this repository folder in
Unity Hub. The editor and Unity Hub are not installed automatically by this
repository.

The built-in Physics module is enabled through `Packages/manifest.json`.
Keyboard driving uses Unity's legacy Input Manager, enabled in the project
settings; no external Unity packages are required. The scripts can be edited in
Visual Studio or VS Code.

## Create the starter scene

After the project opens and Unity finishes importing, select
**Tools > CombatSim > Create Starter Scene**. This creates and saves
`Assets/Scenes/CombatArena.unity`, with a configurable arena, one drivable robot,
and a camera. Press Play to drive:

After replacing the starter scene, select **Game** and press **Play** to view the
robot in the arena. Double-click the Game tab to maximize it if the preview is
too small.

- **W / S**: drive both sides forward / backward at the selected wheel RPM
- **A / D**: turn left / right; with no throttle the tracks drive in opposite
  directions and turn in place

The drive wheels default to **424.4 RPM** with a **45 mm** radius, or about
**2 m/s**. Steering uses differential drive: the inside side slows down instead
of steering the robot like a car. The robot chassis uses a low-friction contact
material so PhysX does not pin the robot in place or fight the drivetrain
model; the drive controller applies its own traction-limited force at the
tracks. It is a simple starting model, not a validated engineering simulator.

## Units and arena size

Arena dimensions and other editable dimensions are specified in millimeters.
The default arena is **2000 mm × 2000 mm** (2 m × 2 m). Select the `Arena`
object in the scene and change **Width Mm** or **Depth Mm** in the Inspector.
Use the component context menu's **Rebuild Arena** command after changing its
dimensions.

Unity physics uses SI units: positions and lengths are converted to meters,
mass is in kilograms, force in newtons, and gravity in meters per second
squared. Keeping that conversion at the boundary avoids mixing millimeters
with Unity's physics calculations.

## Current scope

This starter project provides the arena, basic rigid-body driving, and a scene
generation menu. CAD import, selectable parts and weapons, two-player matches,
and an AI opponent are follow-on features, not implemented yet.
