# Aster Gameplay Server

P0 authoritative gameplay server for a browser TPS. Python owns movement, physics, combat, health,
death, respawn, and snapshots. The browser client is not part of this server-only vertical slice.

## Requirements

- Python 3.13+
- Panda3D 1.10.16+

## Run

```powershell
py -3.13 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -e .
python -m aster_game.app
```

## Verify

```powershell
uv sync --python 3.13 --group dev
uv run --locked pytest -q
uv run --locked ruff check .
```

Health and in-memory server metrics are available at `/healthz` and `/metrics`. WebSocket clients
connect to `/ws`, send `hello` with protocol version `1`, then `join_game`. Omitting `room_id` joins
an available room or creates one. Explicit room IDs only join an existing room.

The protocol is JSON. Client messages are `hello`, `join_game`, `input`, `attack`, `respawn`, and
`ping`. Server messages include `welcome`, `joined`, `snapshot`, gameplay events, `pong`, and
`error`. The detailed wire models live in `aster_game/network/messages.py`.

Example client flow:

```json
{"type":"hello","protocol_version":1}
{"type":"join_game","player_name":"Player One"}
{"type":"input","sequence":1,"move_x":0,"move_z":1,"jump":false,"sprint":false,"yaw":0}
{"type":"attack"}
```

`input` contains intent only; position and velocity are server-owned. `yaw` is degrees, movement
axes are clamped to `[-1, 1]`, and sequences must increase. Repeated movement input is coalesced per
tick while jump press edges are retained. The server acknowledges the latest applied sequence in
each player's snapshot. Messages larger than 16 KiB are rejected by the WebSocket server.

Gameplay events include `state_changed`, `jump_started`, `fall_started`, `fall_impact`, `landing`,
`attack_fired`, `attack_rejected`, `hit`, `projectile_impact`, `damage`, `health_changed`, `death`,
`respawn`, and `command_rejected`.

## World and simulation

- One asyncio task and one independent Bullet world per room.
- 30 Hz fixed simulation tick by default; physics advances using that fixed timestep.
- Y is the vertical axis; character transforms use world coordinates in meters.
- The arena has a floor, perimeter walls, central cover, and a stepped upper platform that makes
  threshold fall damage reachable during play.
- Characters use a server-side Bullet capsule controller. Projectiles use swept-sphere queries so
  fast projectiles do not skip collisions between ticks.
- Simulation state is in memory. No database or external message broker participates in gameplay.

All settings can be overridden with the `ASTER_GAME_` prefix, for example
`ASTER_GAME_TICK_RATE=30` or `ASTER_GAME_PORT=8000`.
