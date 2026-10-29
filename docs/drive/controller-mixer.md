# Controller setup

Open **Controller setup** from Home or Drive. The setup has three steps:

1. **Connect your controller.** Plug it in and select it from the Controller list.
   The green Connected status shows the number of axes and buttons detected.
2. **Map the sticks and buttons.** Enable **Use controller mapping**, then press
   **Detect** beside an input. Move one stick fully or press one button. CombatSim
   assigns that physical control to the input and shows its live value. Repeat for
   each input you need. Control numbers start at 1.
3. **Map inputs to bot controls.** Choose a motor group or Weapon. Set each input's
   weight, offset, and expo, then move the controller and watch the response graph.

The default mix uses Input 1 for throttle, Input 2 for steering, and Input 3 for
the weapon. Fine tune **Reverse** if an axis moves the wrong way. Increase its
**Deadzone** if a stick drifts while centered. The default drive mix combines
throttle and steering for left and right motors; weapon output ranges from 0% to
100%.

The selected physical controller is remembered on this computer. Each bot keeps
its own channel map and output mix, saved automatically when changed. Load a bot
to return to its saved controller setup. A bot without a saved setup starts with
the default mix.

If an input does not respond, confirm the controller is connected, its live bar
moves, mapping is enabled, and the input has a nonzero weight for the selected
bot control.
