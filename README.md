# CombatSim

Unity 6 combat robot simulator using the embedded MuJoCo 3.3.7 Unity plug-in. The default build scene is `Assets/Scenes/CombatArenaMuJoCo.unity`; its physics model is `Assets/CombatSim/Models/CombatRobot500g.xml`.

## Run

1. Open the project in Unity 6. The matching Windows x86-64 MuJoCo DLL is included at `Assets/Plugins/x86_64/mujoco.dll`.
2. Open `Assets/Scenes/CombatArenaMuJoCo.unity` and press **Play**. To recreate it from MJCF, use **Tools > CombatSim > Create MuJoCo Combat Arena**.
3. Drive with **W/S**, turn with **A/D**, and hold **Space** to spin up the drum.
   A USB gamepad uses the left stick; hold the left bumper or primary button to
   spin up the drum.

The MuJoCo model runs at 2,500 Hz with a 0.4 ms simulation step. The 2 m square fighting box has 1 m tall translucent collision walls and a translucent collision roof. Each robot has a 500 g mass: 350 g chassis, four 7.5 g wheels, and a 120 g vertical drum with two teeth. The player's drum target is 30,000 RPM (about 3,142 rad/s), with a 1.5 second commanded spin up and motor torque capped at 0.25 N m. Its estimated rotational energy at full speed is about 572 J. The target robot's drum is unpowered.

References: [MuJoCo Unity plug-in instructions](https://mujoco.readthedocs.io/en/3.3.7/unity.html), [MuJoCo 3.3.7 release](https://github.com/google-deepmind/mujoco/releases/tag/3.3.7).
# CAD robot editor and Fusion 360 import

The Unity editor tools are under **Tools > CombatSim > Hub and Bot Editor**.
The Hub is the project start screen: it creates robot design assets, imports CAD,
opens the MuJoCo practice arena, and can apply the selected design before Play.

## Fusion 360 workflow

1. In Fusion, use **Utilities > Add-ins > Scripts and Add-ins** and add the
   `Fusion360Exporter/CombatSimExport.py` script. Select the `Fusion360Exporter`
   folder as the script location.
2. Run **CombatSimExport** with the robot assembly open. Choose an output folder.
   The exporter writes a separate STL for each solid body and a `manifest.json`
   with body and part names, assembly placement, Fusion material name, density,
   and mass.
3. In Unity, open **Tools > CombatSim > Hub and Bot Editor**, choose **Import
   Fusion Bundle...**, and select that `manifest.json`.
4. Review the body groups and each part's material, density, friction estimate,
   and mass. A body mass override is useful for an extra body whose CAD material
   is unknown. It is divided across that body's included parts, weighted by CAD
   mass when available. The weight display includes the practice robot's stock
   150 g drive and weapon assembly; uncheck CAD parts that duplicate those
   stock mechanisms before applying a full-robot assembly.
5. Choose **Apply Design and Play**. Included parts become MuJoCo mesh geoms on
   fixed bodies attached to the player chassis. The stock chassis geom is
   replaced; the practice arena's drive and weapon mechanisms remain available.

STL and OBJ files can also be imported directly with **Add STL / OBJ Part**.
Those mesh formats do not carry physical material or mass, so set the part's
material, density, friction estimate, and mass in the editor. Fusion bundles use
STL for robust per-body geometry and the manifest for the CAD metadata.

## Material estimates

The editor includes presets for UHMW polyethylene, 7075 aluminum, tool steel,
PA12 nylon, generic nylon, ABS, TPU 95A, platinum-cure silicone 40A and 20A,
carbon-fiber composite, and an unknown/custom option. Density and sliding
friction are editable for every part. Tool steel is a family of grades, so enter
the exact grade in the part name or notes and adjust its preset values when the
grade differs. Unknown materials can use a body or part mass override.

These friction values are starting estimates, not fixed material constants.
Actual friction depends on both contacting surfaces and finish. Imported parts
use their per-geom estimate against the arena floor and walls; MuJoCo combines
equal-priority surfaces using its contact rules.

Extra editor bodies currently attach rigidly to the chassis and contribute their
configured mass and collision mesh. They are fixed to the chassis in this
version. The robot's wheel and weapon joints remain the stock 500 g practice
model. Independent CAD joints, a direct STEP parser, and per-contact-pair
friction tuning are follow-on items.
