# Aster Gameplay Server

P0 authoritative gameplay server for a browser TPS. Python owns movement, physics, combat, health,
death, respawn, and snapshots. The repository includes a small browser test client for exercising
the networked gameplay loop; it is not the production 3D client.

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

Open `http://127.0.0.1:8000/` for the browser gameplay test page. It connects to the local
WebSocket by default, joins a room, predicts local horizontal motion between authoritative
snapshots, and displays both server and client positions. The page sends input at the negotiated
server tick rate and renders through `requestAnimationFrame`; its FPS and server Hz are visible in
the header. Use `WASD` to move, `Shift` to sprint, `Space` to jump, arrow keys or `Q`/`E` to turn,
`F` to fire, and `R` to request a respawn. The arena is a 2D test view, not a production 3D client.

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
- Fixed simulation tick supports 60–120 Hz; the default is 60 Hz and physics advances using that timestep.
- Full authoritative snapshots are sent at the simulation rate by default (60 Hz).
- Y is the vertical axis; character transforms use world coordinates in meters.
- The arena has a floor, perimeter walls, central cover, and a stepped upper platform that makes
  threshold fall damage reachable during play.
- Characters use a server-side Bullet capsule controller. Projectiles use swept-sphere queries so
  fast projectiles do not skip collisions between ticks.
- Simulation state is in memory. No database or external message broker participates in gameplay.

All settings can be overridden with the `ASTER_GAME_` prefix, for example
`ASTER_GAME_TICK_RATE=60` or `ASTER_GAME_PORT=8000`. Tick rates below 60 Hz are rejected.
