# Keyboard and gamepad controls

## Keyboard

| Action | Main bot | Opponent bot |
|---|---|---|
| Forward | W | I |
| Reverse | S | K |
| Turn left | A | J |
| Turn right | D | L |
| Weapon | Hold Space | Hold M |

The main and opponent controls are independent. A and D can turn the player
without W or S; the two wheel sides counter-rotate for a spot turn.

## USB gamepad

The main bot can use a USB joystick/gamepad. Open **Controller setup** and
follow its three steps: connect the device, use **Detect** to assign each stick
or button, then map inputs to drive motor groups or the weapon. The live bars
show which controls are moving. Use **Reverse** if a control acts in the wrong
direction and **Deadzone** if a centered stick drifts.

The selected physical controller is remembered on this computer. Input
assignments and output mixes are saved separately for each loaded bot model and
restored when that model is loaded again.

For analog shaping and channel assignment, see [Controller mixer](controller-mixer.md).
