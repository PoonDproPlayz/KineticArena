---
title: Record and Replay 3D Combat Robot Fights
description: 'Record a KineticArena robot fight, replay it in the 3D arena, move the camera, and review the simulation at different playback speeds.'
---

# Record and review 3D fight replays

KineticArena can save a fight and play it back in the 3D arena.

## Record a fight

1. Open **Arena** and set up the player and opponent bots.
2. Select **Record fight**. Recording works while the simulation is running or
   paused.
3. Select **Stop recording and save** when the fight is over.

KineticArena also saves the recording when you leave the Arena page or close the
app. Each recording is limited to ten minutes. A multiplayer host can record a
fight; clients do not own the simulation state.

## Review and share

Open **Replays** in the top navigation, select a saved fight, then choose
**Load selected replay**. The replay file includes the MuJoCo model used for the
fight, so it can be reviewed later or copied to another computer without
rebuilding the bot setup. Use **Open replay folder** to find or share `.kar`
files.

The replay panel provides a timeline, play/pause, five-second skips, restart,
and playback speed from 0.1x to 4x. **Camera follow** can track either bot or
leave the camera free. In free camera mode, left-drag to orbit, right-drag to
pan, and scroll to zoom.

Replays capture poses for scrubbing and record actuator inputs at each physics
step. New replay files can use **Re-simulate recorded inputs** to run those
commands through the embedded MuJoCo model again. This is useful for checking a
trajectory and reproducing a run in the same simulator build; floating-point
solver differences can still affect results across machines or versions.

Old pose-only replay files remain viewable, but cannot be re-simulated. Replay
recording is limited to ten minutes and stops if its file would exceed 512 MB.
