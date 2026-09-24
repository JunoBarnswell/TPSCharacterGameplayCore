# Network Motion Runtime

## Ownership

Python owns authoritative character state, Bullet collision, movement validation, and replicated
snapshots. Browser prediction runs the same unconstrained acceleration, braking, gait, and rotation
equations with a lightweight replica of the packaged arena collision profile. The replica resolves
capsule-radius AABB contacts, wall sliding, bounded step-up, floor support, slope classification, and
ground snap. It is a prediction approximation; Bullet remains the authority when the two collision
implementations disagree.

## Owner prediction and reconciliation

Each input has a sequence and client tick. `PredictionHistory` retains the solved movement and ground
state after every predicted input. A snapshot identifies `last_processed_input`; reconciliation then
restores the authoritative movement state, removes acknowledged entries, and replays the remaining
commands through the collision replica. Position, yaw, and velocity error plus reconciliation and
large-correction counts are kept for debugging.

The simulation transform is never overwritten by visual smoothing. Small corrections initialize a
position/yaw offset that decays using exponential or linear smoothing. A configured snap mode, position
threshold, or yaw threshold clears the offset immediately. The 2D lab displays the authoritative,
predicted, and visual transforms separately.

## Remote interpolation and clock

`RemoteSnapshotBuffer` stores multiple timestamped transforms and velocities. Interpolation uses
bounded cubic Hermite position curves, shortest-arc yaw interpolation, tangent clamping, overshoot and
visual-speed guards, and teleport detection. Extrapolation is capped in time and speed. Fallback to a
linear segment is an explicit interpolation result mode.

`ServerClockEstimator` estimates server tick phase from snapshot arrival and optional RTT samples.
`AdaptiveInterpolationDelay` tracks snapshot interval, arrival jitter, RTT mean, and RTT variance, then
clamps a responsive delay to configured minimum and maximum values. The simulation rate, snapshot
interval, and browser render rate remain independent; defaults are 60 Hz, 20 Hz, and
`requestAnimationFrame` respectively.

## Wire contract

Protocol v6 carries movement intent, view yaw/pitch, rotation mode, and requested gait. It does not
accept a client-authored position, final character yaw, or hit target. Server firing derives a bounded
aim ray from the latest accepted view angles, raycasts the authoritative Bullet world while ignoring
the owner's capsule, then launches the projectile toward that server-selected contact point. A
snapshot separates the local `owner` profile from `players`, which contains remote animation/interpolation state. Input acknowledgement,
jump availability, and detailed ground-probe diagnostics remain owner-only. The server validates and
serializes one recipient-specific snapshot, then queues the encoded JSON text for its WebSocket writer.
Protocol versions 1–5 are rejected. The static collision profile is versioned and sent in Welcome.

## Limits

- Cross-runtime golden and 10/30/60 second drift tests cover the shared unconstrained movement
  equations. They do not imply that floating-point Panda3D Bullet contacts are deterministic in JS.
- The browser collision profile contains static floor, box, and ramp geometry; moving/dynamic body
  collision and full Bullet ghost-sweep edge behavior are not reproduced.
- Lag compensation, server rewind, and prediction of projectile hits are not implemented.
- Snapshot byte metrics report the largest and average recipient payload in the latest tick and
  accepted enqueue payload bytes over the last second. The counts include snapshots that may later be
  coalesced out of a session queue and exclude WebSocket/TCP framing and kernel buffering.
- See [`motion-debugging.md`](motion-debugging.md) for the lab diagnostics and network simulation.
