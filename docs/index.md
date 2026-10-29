# CombatSim

CombatSim is a desktop practice simulator for small combat robots. It combines
MuJoCo physics, a robot workshop for Fusion 360 assemblies, keyboard and USB
gamepad control, telemetry, and direct LAN matches.

## Get up and running

1. [Install CombatSim](getting-started.md) on Windows.
2. [Drive the stock robot](drive/first-drive.md) to learn the controls.
3. [Import your Fusion 360 assembly](configuration/fusion-import.md) when you
   are ready to practice with your own bot.

## Guides

| I want to… | Guide |
|---|---|
| Set up the app and launch it | [Installation](getting-started.md) |
| Drive, turn, and read telemetry | [First drive](drive/first-drive.md) |
| Connect a USB gamepad or change its mix | [Controller mixer](drive/controller-mixer.md) |
| Configure parts, wheels, forks, and axes | [Parts and mechanisms](configuration/parts-and-mechanisms.md) |
| Set motor ratings and gear ratios | [Drive and weapon motors](configuration/motors.md) |
| Estimate the mass of a printed part | [Printed part mass](configuration/printed-parts.md) |
| Practice against the built-in opponent | [Opponent AI](drive/opponent-ai.md) |
| Play over a local network | [LAN multiplayer](drive/multiplayer.md) |
| Adjust the arena and lighting | [Arena and graphics](settings/arena-graphics.md) |

## What the current build includes

- A 2,500 Hz MuJoCo simulation with standard gravity (`9.80665 m/s²`).
- A configurable arena with center-line or diagonal starting positions.
- Fusion bundle import, per-body setup, wheel and weapon motor models, and
  gravity-driven fork modules.
- A classic renderer and an optional experimental Filament PBR renderer.
- LAN matches where the host runs the shared physics simulation.

Download the [latest Windows release](https://github.com/PoonDproPlayz/KineticArena/releases/latest)
to run CombatSim without setting up a C++ build environment. The ZIP has one
application executable to launch plus its required runtime files. To modify or
build the code, see [Build from source](development/building.md). Release assets
and versioning are described in [Release packaging](development/releases.md).

Match timers, winner declaration, and per-part damage are also not implemented
in the current build.
