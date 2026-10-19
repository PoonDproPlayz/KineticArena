# CombatSim project handoff

Updated: 2026-10-19 (latest user direction: unable to test right now)

This note is for a future ChatGPT/Codex session continuing work in this folder. Read this before changing files. The Unity project has a large uncommitted worktree; preserve it and inspect the current state instead of resetting or replacing files.

## User's goal

Build a combat robot practice simulator in Unity using MuJoCo physics rather than PhysX. The robot is in the 500 g class. The user wants to import a Fusion 360 robot design, edit material and mass properties, set weapon RPM, connect a USB gamepad, and practice driving. The user prefers having the assistant make the changes and then test them in Unity when Unity Hub is working. Do not claim an interactive Unity test succeeded unless it actually did.

## Current project

- Unity editor version: `6000.6.6f1` (`ProjectSettings/ProjectVersion.txt`).
- MuJoCo Unity package: local package `org.mujoco` version `3.3.7` at `Packages/org.mujoco`.
- Main MuJoCo scene: `Assets/Scenes/CombatArenaMuJoCo.unity`.
- Main MJCF model: `Assets/CombatSim/Models/CombatRobot500g.xml`.
- Build settings currently point to the MuJoCo scene.
- The model uses a 0.0004 s timestep (2,500 Hz), a 2 m square arena, 1 m tall translucent collision walls, and a translucent collision roof.
- Each practice robot is modeled at 500 g: 350 g chassis, four 7.5 g wheels, and a 120 g vertical drum. The player's weapon target is 30,000 RPM with a 1.5 s spin-up and 0.25 N m actuator force cap. The target robot's weapon is unpowered.
- Keyboard: W/S drive forward/reverse, A/D steer, Space spins up the weapon. A/D alone commands an in-place turn. The controller also supports a USB gamepad left stick and weapon button.
- Editor Hub and Bot Editor: **Tools > CombatSim > Hub and Bot Editor**.

## CAD/Fusion workflow

The Fusion exporter is in `Fusion360Exporter/CombatSimExport.py`, with `config.json` alongside it. In Fusion, open **Utilities > Add-ins > Scripts and Add-ins**, add the `Fusion360Exporter` folder from this project, then run `CombatSimExport` with the robot design open. Choose an output folder. It creates `<design>_MuJoCo/Parts/*.stl` and `manifest.json`.

In Unity, use **Tools > CombatSim > Hub and Bot Editor > Import Fusion Bundle...** and choose `manifest.json`. Review imported parts/materials/mass, then use **Apply Design and Play**. Direct STEP import is not implemented; this workflow uses per-solid-body STL plus manifest metadata. STL/OBJ can also be added individually, but those files do not contain Fusion material or mass metadata. Extra editor bodies are fixed to the chassis in the current version. The imported design replaces the stock chassis geometry while stock drive and weapon mechanisms remain; avoid duplicating them and account for the retained 150 g in the weight display.

Material presets include UHMW, 7075 aluminum, tool steel, PA12, nylon, ABS, TPU 95A, platinum-cure silicone 40A/20A, carbon fiber, and unknown/custom. Density and friction estimates can be edited. These are starting estimates, not universal contact constants.

## Current Fusion bundle import error

The user's bundle is at `C:\Users\nabha\Desktop\Combat robotics\Sim\Sparrow_files\Sparrow_v157_MuJoCo`. The manifest is next to its `Parts` directory and its relative paths are correct, but the bundle is incomplete: the manifest references 95 STL files and only 87 exist. Missing entries are `Parts/0000_Body123.stl`, `Parts/0001_Body124.stl`, `Parts/0008_Body166.stl`, `Parts/0010_Body168.stl`, `Parts/0013_Body126.stl`, `Parts/0014_Body152.stl`, `Parts/0073_Body1.stl`, and `Parts/0077_Body2.stl` (last two are in the Weapon and Carbon fibre plates and covers occurrences). The user needs to rerun the Fusion export and ensure every referenced STL is created in the `Parts` folder before importing again; do not simply drop those manifest records because that would omit CAD geometry/mass.

The importer was changed to preflight all referenced mesh paths before creating any Unity assets and report every missing part/path together. The Fusion exporter now verifies that each export actually produced a non-empty binary STL before writing its metadata. Fusion Python syntax and the Unity Editor C# assembly response-file compile both passed after these edits. The actual Fusion export still needs to be rerun inside Fusion.

The user currently cannot test the Unity/Fusion import and explicitly asked to update this handoff instead. Do not make progress depend on them running tests right now or repeat the same request. Continue with useful static inspection/validation and resume interactive tests only when they say they are able to test.

## Unity launch/license and MuJoCo runtime problems

The user's latest screenshot shows a Windows dialog titled `Unity.Licensing.Client.exe - Application Error` with exception code `0xE0434352` and a faulting address. This is a .NET CLR exception code; by itself it does not name the underlying exception or root cause. Microsoft says the HRESULT and exception details are needed to diagnose it: [Microsoft Learn: CLR Exception E0434352](https://learn.microsoft.com/en-us/shows/inside/e0434352).

Local logs provide a more nuanced picture than the screenshot alone. Unity Hub launched Unity 6000.6.6f1 for this project at about 2026-10-19 23:09 local time. The Editor log recorded `Code 10 while verifying Licensing Client signature`, then a successful licensing handshake, access token, resolved entitlement details, and Unity Personal license update. A Unity employee has described this Code 10 signature-verification report as a false positive in [Unity's discussion](https://discussions.unity.com/t/licensing-client-error-code-10-while-verifying-licensing-client-signature/945257); it is not enough to explain the separate Windows CLR exception. The Licensing Client log did not show the exception type/stack. So do not conflate the screenshot with the MuJoCo failure or claim its root cause is known.

A project-relative `Logs/Editor.log` shows Unity ticking the MuJoCo scene and repeatedly recording `NullReferenceException: Failed to create Mujoco runtime` at `Packages/org.mujoco/Runtime/Components/MjScene.cs:350` (`Model` or `Data` is null). At the last inspection the log had grown to about 55 MB with roughly 78,000 repeated copies. This is a separate, concrete Play-mode blocker. The actual native model-load error is not present in the captured log. The scene's `MjGlobalSettings.DebugFileName` is blank; set it to a diagnostic XML name or add targeted error logging, stop the repeated runtime error loop, regenerate the scene model, then inspect the emitted/generated MJCF and MuJoCo parse result. The current standalone MJCF was parsed/stepped in a prior native smoke test, but the Unity plugin generates MJCF from scene components, so that does not validate the Unity-generated model.

An earlier console screenshot showed thousands of MuJoCo package test compile errors referencing NUnit/Unity TestTools, and the first package import showed two package API compile errors. The current Editor log later shows a successful Tundra C# assembly build without compile errors, so treat those old compile errors as historical unless they recur. An earlier headless startup also timed out waiting for licensing; the later Hub-launched Editor session did connect successfully. No successful in-Unity physics session has been verified yet.

Useful Windows logs from Unity's documentation:

- Licensing client: `%LOCALAPPDATA%\Unity\Unity.Licensing.Client.log`
- Unity Editor: `%LOCALAPPDATA%\Unity\Editor\Editor.log`
- Unity Hub: `%APPDATA%\UnityHub\logs\info-log.json`

The user already has these local logs available; inspect them before asking the user to collect them again. Unity's reference for log locations and client troubleshooting is [Unity licensing client troubleshooting](https://docs.unity.com/en-us/licensing-server/troubleshooting-client). If the CLR dialog recurs, capture the matching Windows Application Error event / crash details to get the managed exception type and HRESULT. Launch through Unity Hub interactively. Do not keep retrying headless launches as a substitute for diagnosing the Licensing Client crash.

## Validation and worktree notes

- Unity's project-relative `Logs/Editor.log` records a successful Tundra C# assembly build with obsolete-API warnings from embedded MuJoCo. This is not a successful physics runtime test.
- A prior native MuJoCo smoke test loaded the arena/model with an STL mesh and mass/friction properties, then stepped the simulation. This was not an in-Unity physics test.
- At the latest inspection, two Unity Editor processes started around the Hub launch were still present with no visible window title; avoid killing them or editing the project while the user may have an active Editor session. The project log was actively growing from the MuJoCo null-runtime loop. If an Editor window is available, stop Play mode before further testing.
- The current worktree contains many modified and untracked project files, including the MuJoCo package, scene/model, controller, Hub/editor scripts, and Fusion exporter. There is no handoff commit; preserve these changes.
- If code changes are made, inspect the existing scene/model and avoid regenerating/overwriting user assets without need. After Unity Hub is usable, open the project with Unity 6.6, check Console, open `CombatArenaMuJoCo.unity`, press Play, and verify controls, arena visibility, robot mass/model, weapon direction and spin-up.
- A potential requirement to verify in the live scene: confirm the player and target actually face toward one another. The current MJCF places them on opposite X sides, but both appear to use the identity body orientation, so heading should be checked rather than assumed correct.

## Immediate next steps

1. Continue diagnosing the `MjScene` null runtime using static inspection and, when possible, a generated-MJCF debug file or targeted native error-buffer logging. Avoid adding more repeated runtime error spam.
2. Continue static validation of the incomplete Fusion bundle/exporter changes; the current bundle needs the eight missing STL files regenerated in Fusion before import can succeed.
3. If the Licensing Client CLR error recurs, capture Windows Application Error details and correlate them with the three Unity logs above. Do not assume the editor's logged signature Code 10 is the cause: its subsequent license handshake succeeded.
4. When the user is able to test, verify that MuJoCo initializes, then check arena visibility, player/target facing, controls, weapon direction and spin-up in Play mode. Also run an end-to-end Fusion export/import test.
5. Continue remaining design/import/controller improvements based on available static evidence, without waiting for a user test.
