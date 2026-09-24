# Aster Character Motion Engine

Authoritative Python gameplay server and browser Character Motion Lab for a Web TPS. Python owns
movement, physics, combat, health, death, respawn, and snapshots. Browser motion modules implement
matching owner prediction, input replay, visual correction smoothing, remote interpolation, and
animation blend semantics. The canvas lab is not a production 3D client and does not play skeletal
animations.

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
node --test tests/test_web_motion.mjs
```

Health and in-memory server metrics are available at `/healthz` and `/metrics`. WebSocket clients
connect to `/ws`, send `hello` with protocol version `3`, then `join_game`. Omitting `room_id` joins
an available room or creates one. Explicit room IDs only join an existing room.

Open `http://127.0.0.1:8000/` for the Character Motion Lab. It predicts using the same acceleration,
braking, air steering, and rotation equations, restores authoritative snapshots, replays unacknowledged
input history, and smooths only the visual correction. Remote players render from a tick-buffered
Hermite interpolation path. Simulation runs at 60 Hz by default; replication is independently
configurable and defaults to 20 Hz (`snapshot_interval_ticks: 3`); browser rendering uses
`requestAnimationFrame`. The page reports movement channels, floor sample, aim/facing, ACK/correction,
simulation/snapshot/render rates, ping/jitter, and pending input count. Use the Network Simulation
sliders to add application-level latency, jitter, and packet loss. Use `WASD`, `Shift`, `Space`,
left/right or `Q`/`E` to turn the view, up/down to change view pitch, `F` to fire, and `R` to respawn.
The arena remains a 2D test view without a production skeleton runtime.

The protocol is JSON. Client messages are `hello`, `join_game`, `input`, `attack`, `respawn`, and
`ping`. Server messages include `welcome`, `joined`, `snapshot`, gameplay events, `pong`, and
`error`. The detailed wire models live in `aster_game/network/messages.py`.

Protocol v3 example client flow:

```json
{"type":"hello","protocol_version":3}
{"type":"join_game","player_name":"Player One"}
{"type":"input","sequence":1,"client_tick":1,"move_x":0,"move_z":1,"jump":false,"requested_gait":"run","view_yaw":0,"view_pitch":0,"rotation_mode":"orient_to_movement"}
{"type":"attack"}
```

`input` contains intent only; position, velocity, and character facing are server-owned. `requested_gait`
is `walk`, `run`, or `sprint`; snapshots separately report requested and speed-derived actual gait.
Movement tuning includes piecewise acceleration, braking, and turn-speed curves. `view_yaw` and
`view_pitch` are degrees; movement axes are clamped to `[-1, 1]`, and sequences must increase.
Repeated movement input is coalesced per tick while jump press edges are retained. The server
acknowledges the latest applied sequence in each player's snapshot. Protocol v3 intentionally removes
the old `sprint` boolean, client-authored `yaw`, and mutually exclusive character `state` fields; no v2
or v1 aliases are kept. Messages larger than 16 KiB are rejected by the WebSocket server.

Gameplay events include `motion_state_changed`, `jump_started`, `rising`, `apex_reached`,
`fall_started`, `fall_impact`, `landing_started`, `landed`, `attack_fired`, `attack_rejected`, `hit`,
`hit_reaction`, `projectile_impact`, `damage`, `health_changed`, `death`, `respawn`, and
`command_rejected`.

## World and simulation

- One asyncio task and one independent Bullet world per room.
- Fixed simulation tick supports 60–120 Hz; the default is 60 Hz and physics advances using that timestep.
- Replication frequency is set by `snapshot_interval_ticks`; default 3 means 20 Hz at the default 60 Hz simulation.
- Client render rate is independent of server simulation and snapshot rates.
- Y is the vertical axis; character transforms use world coordinates in meters.
- The arena has a floor, perimeter walls, central cover, and a stepped upper platform that makes
  threshold fall damage reachable during play.
- Characters use a server-side Bullet capsule controller. Projectiles use swept-sphere queries so
  fast projectiles do not skip collisions between ticks.
- Simulation state is in memory. No database or external message broker participates in gameplay.

All settings can be overridden with the `ASTER_GAME_` prefix, for example
`ASTER_GAME_TICK_RATE=60` or `ASTER_GAME_PORT=8000`. Tick rates below 60 Hz are rejected.

## Motion architecture

See [`docs/character-motion-runtime.md`](docs/character-motion-runtime.md) for motion-state channels,
solver order, protocol-v2 ownership, prediction/reconciliation, interpolation, and runtime boundaries.

The current browser runtime provides a two-dimensional blend-space weight model, additive aim and hit
layers, inertialized blend weights, semantic orientation-warp angle, trajectory samples, and bounded
motion/pose history. There is no production skeleton/pose backend. Real Foot IK, Root Motion playback,
Motion Warping, Pose Search, and Motion Matching remain future work.
