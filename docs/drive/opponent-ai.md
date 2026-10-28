# Opponent AI

The Drive screen includes a basic aim-and-ram opponent. It turns toward the
player, drives forward, and activates its weapon when one is assigned.

- Use **Aim and ram AI** to turn the AI on or off during a match.
- Adjust **Aggression** to scale its drive speed and turn response.
- At simulation start, the AI waits two seconds before taking control. The
  delay uses simulation time, so a paused match does not consume it.

If the opponent has no left and right wheel drives assigned, the AI cannot
move; configure its drivetrain in **Opponent bot** first. The AI is a simple
practice opponent, not a strategy or damage-aware controller.

You can switch to manual opponent control at any time. Its keyboard controls
are I/K to drive, J/L to turn, and M to spin the weapon.
