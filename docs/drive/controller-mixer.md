# Controller mixer

Open **Controller mixer** from Home or Drive. The mixer maps raw USB joystick
inputs to drive-motor and weapon outputs.

## Set up a controller

1. Connect the controller before opening or refreshing the mixer.
2. Select it in **Input sources**.
3. Enable the custom joystick mixer.
4. Add or remove input channels as needed, then choose a source for each one.
5. Move a stick or press a button and watch **Live** to identify its channel.
6. Assign each channel to the drive groups or weapon output and tune its mix.

Each mix line has a weight, offset, and expo. The graph shows the response for
the selected channel; drag its control point to edit the curve. Other inputs
remain at their current values while you tune that line. Reverse and deadzone
are configured for each input channel.

The default RC tank mix uses left = Ch1 − Ch2, right = Ch1 + Ch2, and weapon =
Ch3. CombatSim clamps drive outputs to the range −100% to +100% and weapon
output to 0% to 100%.

If a control does not respond, check that the right joystick is selected, its
channel moves in the live display, the channel is mapped to the desired output,
and its weight is not zero.
