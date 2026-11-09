---
title: LAN Multiplayer Combat Robot Simulator
description: 'Host or join a KineticArena combat robot practice match over a local network, configure the host, and connect another player.'
---

# LAN multiplayer practice matches

LAN multiplayer connects two KineticArena instances directly. The host runs the
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
4. If Windows asks, allow KineticArena through the firewall on private networks.

## Join a match

1. Open **LAN Multiplayer** on the second computer.
2. Enter the host's IPv4 address, port, and session key.
3. Choose **Join match** and wait for both apps to report ready.

Each player uses W/A/S/D and Space for their own robot. On one computer, only
the focused KineticArena window reads local keyboard/gamepad input. Drive telemetry
shows the local player's weapon and wheel RPM. Axis visibility is local to each
player's own bot.

## Test on one computer

Open KineticArena twice. In the first window, host the match. In the second, join
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

## Online service setup status

The multiplayer window includes **Set up online service**. It opens the bundled
one-time Cloudflare Worker deployment flow; the project owner creates or signs
in to a free Cloudflare account in the browser and approves deployment. Node.js
is installed through WinGet if it is missing. Players will not need Cloudflare
accounts once an online transport is available.

This setup currently deploys only the signaling Worker. KineticArena's match
connection is still TCP and only works when the computers can reach each other
directly. Internet host/join codes are not available until the WebRTC data
channel transport is implemented. The in-app message calls this out so deploying
the service is not mistaken for enabling WAN matches.
