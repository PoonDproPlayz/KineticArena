const ROOM_LIFETIME_MS = 10 * 60 * 1000;
const MAX_MESSAGE_BYTES = 16 * 1024;
const MAX_DESCRIPTION_BYTES = 12 * 1024;
const MAX_CANDIDATE_BYTES = 4 * 1024;
const CREATE_LIMIT = 8;
const CREATE_WINDOW_MS = 60 * 60 * 1000;

function jsonResponse(value, status = 200, extraHeaders = {}) {
  return new Response(JSON.stringify(value), {
    status,
    headers: {
      "Content-Type": "application/json; charset=utf-8",
      "Cache-Control": "no-store",
      ...extraHeaders,
    },
  });
}

function errorResponse(status, message, extraHeaders = {}) {
  return jsonResponse({ error: message }, status, extraHeaders);
}

function randomToken(bytes = 32) {
  const raw = new Uint8Array(bytes);
  crypto.getRandomValues(raw);
  let binary = "";
  for (const value of raw) binary += String.fromCharCode(value);
  return btoa(binary).replaceAll("+", "-").replaceAll("/", "_").replace(/=+$/u, "");
}

async function sha256Hex(value) {
  const digest = await crypto.subtle.digest("SHA-256", new TextEncoder().encode(value));
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, "0")).join("");
}

function equalHex(left, right) {
  if (typeof left !== "string" || typeof right !== "string" || left.length !== right.length)
    return false;
  let difference = 0;
  for (let i = 0; i < left.length; i++) difference |= left.charCodeAt(i) ^ right.charCodeAt(i);
  return difference === 0;
}

function bearerToken(request) {
  const value = request.headers.get("Authorization") || "";
  const match = /^Bearer ([A-Za-z0-9_-]{40,64})$/u.exec(value);
  return match ? match[1] : "";
}

function validSignalMessage(message) {
  if (typeof message !== "string" || new TextEncoder().encode(message).byteLength > MAX_MESSAGE_BYTES)
    return false;
  let packet;
  try {
    packet = JSON.parse(message);
  } catch {
    return false;
  }
  if (!packet || typeof packet !== "object" || Array.isArray(packet)) return false;
  if (packet.type === "description") {
    return packet.description &&
      (packet.description.type === "offer" || packet.description.type === "answer") &&
      typeof packet.description.sdp === "string" &&
      packet.description.sdp.length <= MAX_DESCRIPTION_BYTES;
  }
  if (packet.type === "candidate") {
    const candidate = packet.candidate;
    return candidate && typeof candidate === "object" &&
      typeof candidate.candidate === "string" &&
      candidate.candidate.length <= MAX_CANDIDATE_BYTES &&
      (candidate.sdpMid === undefined || candidate.sdpMid === null ||
        typeof candidate.sdpMid === "string") &&
      (candidate.sdpMid === undefined || candidate.sdpMid === null ||
        candidate.sdpMid.length <= 256) &&
      (candidate.sdpMLineIndex === undefined || candidate.sdpMLineIndex === null ||
        (Number.isInteger(candidate.sdpMLineIndex) && candidate.sdpMLineIndex >= 0 &&
         candidate.sdpMLineIndex <= 256));
  }
  return packet.type === "connected" || packet.type === "leave";
}

export default {
  async fetch(request, env) {
    const url = new URL(request.url);
    if (request.method === "GET" && url.pathname === "/health")
      return jsonResponse({ service: "kineticarena-signaling", status: "ok" });

    if (request.method === "POST" && url.pathname === "/v1/rooms") {
      const ip = request.headers.get("CF-Connecting-IP") || "unknown";
      const rateId = env.CREATE_LIMITS.idFromName(await sha256Hex(ip));
      const allowed = await env.CREATE_LIMITS.get(rateId).fetch("https://limits/take", {
        method: "POST",
      });
      if (!allowed.ok) return allowed;

      const roomId = env.ROOMS.newUniqueId();
      const result = await env.ROOMS.get(roomId).fetch("https://room/create", {
        method: "POST",
      });
      if (!result.ok) return errorResponse(503, "Could not create a match room.");
      const credentials = await result.json();
      const id = roomId.toString();
      return jsonResponse({
        roomId: id,
        hostToken: credentials.hostToken,
        inviteCode: `${id}.${credentials.guestToken}`,
        expiresInSeconds: Math.floor(ROOM_LIFETIME_MS / 1000),
        signalingUrl: `${url.origin}/v1/rooms/${id}/signal`,
      }, 201);
    }

    const match = /^\/v1\/rooms\/([A-Za-z0-9_-]{20,128})\/signal$/u.exec(url.pathname);
    if (request.method === "GET" && match) {
      let roomId;
      try {
        roomId = env.ROOMS.idFromString(match[1]);
      } catch {
        return errorResponse(404, "Match room not found or expired.");
      }
      const signalUrl = new URL("https://room/v1/rooms/signal");
      signalUrl.search = url.search;
      return env.ROOMS.get(roomId).fetch(new Request(signalUrl, request));
    }

    return errorResponse(404, "Not found.");
  },
};

export class SignalingRoom {
  constructor(ctx) {
    this.ctx = ctx;
  }

  async fetch(request) {
    const url = new URL(request.url);
    if (request.method === "POST" && url.pathname === "/create") {
      const existing = await this.ctx.storage.get("room");
      if (existing && existing.expiresAt > Date.now())
        return errorResponse(409, "Room already exists.");
      const hostToken = randomToken();
      const guestToken = randomToken();
      const expiresAt = Date.now() + ROOM_LIFETIME_MS;
      await this.ctx.storage.put("room", {
        hostHash: await sha256Hex(hostToken),
        guestHash: await sha256Hex(guestToken),
        expiresAt,
      });
      await this.ctx.storage.setAlarm(expiresAt);
      return jsonResponse({ hostToken, guestToken });
    }

    if (request.method !== "GET" || url.pathname !== "/v1/rooms/signal" ||
        request.headers.get("Upgrade")?.toLowerCase() !== "websocket")
      return errorResponse(404, "Not found.");

    const role = url.searchParams.get("role");
    if (role !== "host" && role !== "guest")
      return errorResponse(400, "Role must be host or guest.");
    const token = bearerToken(request);
    if (!token) return errorResponse(401, "A room access token is required.");

    const room = await this.ctx.storage.get("room");
    if (!room || room.expiresAt <= Date.now())
      return errorResponse(404, "Match room not found or expired.");
    const expected = role === "host" ? room.hostHash : room.guestHash;
    if (!equalHex(expected, await sha256Hex(token)))
      return errorResponse(403, "Invalid room access token.");

    const pair = new WebSocketPair();
    const client = pair[0];
    const server = pair[1];
    for (const oldSocket of this.ctx.getWebSockets()) {
      if (oldSocket.deserializeAttachment()?.role === role) {
        try { oldSocket.close(4001, "Replaced by a reconnecting peer."); } catch { /* already closed */ }
      }
    }
    server.serializeAttachment({ role });
    this.ctx.acceptWebSocket(server);

    const peers = new Map();
    for (const socket of this.ctx.getWebSockets()) {
      if (socket.readyState !== WebSocket.OPEN) continue;
      const attachment = socket.deserializeAttachment();
      if (attachment?.role === "host" || attachment?.role === "guest")
        peers.set(attachment.role, socket);
    }
    if (peers.has("host") && peers.has("guest")) {
      for (const socket of peers.values()) {
        try { socket.send(JSON.stringify({ type: "peer-ready" })); } catch { /* peer reconnects */ }
      }
    }
    return new Response(null, { status: 101, webSocket: client });
  }

  async webSocketMessage(socket, message) {
    if (typeof message !== "string" || !validSignalMessage(message)) {
      try { socket.close(1008, "Invalid signaling message."); } catch { /* already closed */ }
      return;
    }
    const senderRole = socket.deserializeAttachment()?.role;
    const otherRole = senderRole === "host" ? "guest" : "host";
    const peer = this.ctx.getWebSockets().find(
      (candidate) => candidate.deserializeAttachment()?.role === otherRole);
    if (peer) {
      try { peer.send(message); } catch { /* a disconnect can race a send */ }
    }
  }

  async alarm() {
    for (const socket of this.ctx.getWebSockets()) {
      try { socket.close(1001, "Match room expired."); } catch { /* already closed */ }
    }
    await this.ctx.storage.deleteAll();
  }
}

export class CreateRateLimit {
  constructor(ctx) {
    this.ctx = ctx;
  }

  async fetch(request) {
    if (request.method !== "POST") return errorResponse(404, "Not found.");
    const now = Date.now();
    const events = (await this.ctx.storage.get("events") || [])
      .filter((time) => now - time < CREATE_WINDOW_MS);
    if (events.length >= CREATE_LIMIT) {
      const retryAfter = Math.max(1, Math.ceil((events[0] + CREATE_WINDOW_MS - now) / 1000));
      return errorResponse(429, "Room creation limit reached; try again later.", {
        "Retry-After": String(retryAfter),
      });
    }
    events.push(now);
    await this.ctx.storage.put("events", events);
    await this.ctx.storage.setAlarm(events[0] + CREATE_WINDOW_MS);
    return new Response(null, { status: 204 });
  }

  async alarm() {
    const now = Date.now();
    const events = (await this.ctx.storage.get("events") || [])
      .filter((time) => now - time < CREATE_WINDOW_MS);
    if (events.length) {
      await this.ctx.storage.put("events", events);
      await this.ctx.storage.setAlarm(events[0] + CREATE_WINDOW_MS);
    } else {
      await this.ctx.storage.deleteAll();
    }
  }
}
