# KineticArena signaling Worker

This Cloudflare Worker is the small coordination service for a future direct
WAN multiplayer transport. It creates short-lived, two-player rooms and forwards
only WebRTC offer/answer and ICE candidate messages. It does not run MuJoCo,
receive fight snapshots, transfer bot packages, or relay game traffic.

## Deploy

From KineticArena, choose **Set up online service** in the Multiplayer window.
This opens the bundled deployment script in PowerShell. It installs Node.js LTS
through WinGet if needed, installs Wrangler, and opens Cloudflare login in your
browser. Create/sign in to a free account and approve the deployment. The Worker
is published to the account's `workers.dev` subdomain. This is a one-time
project-owner setup; players do not need Cloudflare accounts. The script saves
the service URL as `worker-url.txt`; `/health` should return
`{"service":"kineticarena-signaling","status":"ok"}`.

For local development, run `npm install`, then `npm run dev`.

## API outline

- `GET /health` returns service health.
- `POST /v1/rooms` creates a 10-minute room and returns a private host token,
  an invite code containing the guest token, and the WebSocket signaling URL.
- `GET /v1/rooms/{roomId}/signal?role=host|guest` upgrades to a WebSocket. Pass
  the matching room token in `Authorization: Bearer ...`. Once both players are
  connected, each receives `{"type":"peer-ready"}`. The room forwards bounded
  `description`, `candidate`, `connected`, and `leave` JSON messages between
  the two peers.

The Worker limits room creation by source IP, permits one connection per role,
stores only token hashes, caps signaling payloads, and removes room state when
the room expires. Cloudflare's edge still processes connection metadata, and
the other player will learn the network candidates needed for direct
connectivity.

## Current integration status

The desktop app's existing `LanSession` is still TCP-only. The app includes the
one-time Worker setup flow, but the Worker does not make WAN matches available
until the app has a WebRTC/ICE data-channel transport and connects its
negotiation messages to this API. No game traffic should be sent through this
Worker. Account creation and Cloudflare's authorization/terms must be completed
by the account owner in the browser; the app cannot sign up or consent for them.
