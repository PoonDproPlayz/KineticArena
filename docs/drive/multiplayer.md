# LAN multiplayer

LAN multiplayer connects two CombatSim instances directly. The host runs the
MuJoCo simulation; the joining player sends control input and receives physics
snapshots. It currently supports one joining player and requires a reachable
network route between the computers.

## Before the match

Both players should load and apply a file-backed Fusion bot. The host's bot
package is sent to the guest during connection; the guest does not need a copy
of the host's Fusion bundle on disk.

## Host a match

1. Open **LAN Multiplayer** from Home or Drive.
2. Keep the default port `29811`, then choose **Host match**.
3. Share the displayed session key, port, and the host computer's local IPv4
   address. In PowerShell, `ipconfig` shows the active Wi-Fi or Ethernet
   address.
4. If Windows asks, allow CombatSim through the firewall on private networks.

## Join a match

1. Open **LAN Multiplayer** on the second computer.
2. Enter the host's IPv4 address, port, and session key.
3. Choose **Join match** and wait for both apps to report ready.

Each player uses W/A/S/D and Space for their own robot. On one computer, only
the focused CombatSim window reads local keyboard/gamepad input. Drive telemetry
shows the local player's weapon and wheel RPM. Axis visibility is local to each
player's own bot.

## Test on one computer

Open CombatSim twice. In the first window, host the match. In the second, join
using `127.0.0.1`, port `29811`, and the same session key. Load a local bot in
both windows before connecting.

## Privacy and limits

The bot package uses an AES-GCM encrypted TCP connection and is loaded into
memory instead of extracted to a folder. Temporary package buffers are cleared
after model loading. The running physics model still contains geometry needed
to render and simulate the bot, so a determined peer could inspect process
memory or reconstruct geometry. This is not a guarantee of CAD secrecy or an
anti-cheat system.

There is no internet relay, NAT traversal, router port-forwarding setup, or host
migration. The two computers must be able to connect directly over the LAN or
another network with a route between them.
