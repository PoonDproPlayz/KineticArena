# First drive

Start from **Home → Practice drive**. The stock practice scene loads two bots
facing each other. The selected arena start layout is applied when the match
starts and when you reset it.

## Keyboard controls

| Key | Main bot | Opponent bot |
|---|---|---|
| Forward / reverse | **W / S** | **I / K** |
| Turn left / right | **A / D** | **J / L** |
| Weapon | Hold **Space** | Hold **M** |

With no throttle, A or D counter-rotates the two wheel sides so the robot can
turn in place. With throttle, steering slows one side to make an arc. A USB
gamepad can control the main bot; see [Controller mixer](controller-mixer.md).

Use **Pause** to stop the simulation and **Reset** to return the bots to the
selected starting positions. Reset also clears their simulated motion.

## Telemetry

The Drive panel reports actual weapon RPM and each assigned wheel's RPM, along
with left/right averages. Wheel status indicates whether a wheel is gripping,
slipping, or airborne. The weapon's actual RPM can be lower than its configured
target while it accelerates or when the simulated motor is loaded.

The optional **Manual wheel RPM override** commands a wheel speed directly. It
bypasses the motor torque model and is useful for a controlled speed check; it
does not represent motor-limited driving.

## Move and rotate a bot

Select **Move bot** to show the XYZ arrows on the selected robot. Drag an arrow
to move along that axis. Press **R** to switch to rotation rings, then drag a
ring to rotate around its axis. Press **R** again to return to move arrows.
Right-drag the arena to pan the view; left-drag empty arena to orbit.

Manual placement is disabled for the joining player during a LAN match because
the host owns the shared simulation state.
