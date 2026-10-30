# Fight replays

CombatSim can save a fight and play it back in the 3D arena.

## Record a fight

1. Open **Arena** and set up the player and opponent bots.
2. Select **Record fight**. Recording works while the simulation is running or
   paused.
3. Select **Stop recording and save** when the fight is over.

CombatSim also saves the recording when you leave the Arena page or close the
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

Replays capture up to 120 poses per second, depending on the simulation
timestep, and stop after ten minutes. They are for visual review; playback does
not re-run the physics or let you change the fight.
