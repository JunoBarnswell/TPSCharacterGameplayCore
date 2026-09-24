# Character Motion Runtime Design

## Scope and ownership

Python remains authoritative for collision, movement state, health, and life state. Browser code runs
the shared movement equations against a lightweight replica of the static arena collision profile,
then restores authoritative state and replays unacknowledged inputs. Panda3D Bullet remains the final
authority; the browser AABB/plane/ramp solver is a prediction approximation. Collision correction is
smoothed only in the visual transform. Remote characters are rendered from a time-delayed snapshot
buffer.

The fixed simulation rate, replication rate, and browser render rate are independent. The default
server tick remains 60 Hz; the default snapshot interval becomes three ticks (20 Hz). The browser
uses `requestAnimationFrame` and can render at its display rate.

## State channels

`CharacterMovementState` owns velocity, acceleration, desired movement, floor sample, movement mode,
gait, distance-driven gait phase, facing/view rotations, rotation mode, and locomotion phase.
`Character` separately owns locomotion, upper-body action, additive reaction, full-body override, and
life override animation channels plus `LifeState`. A hit reaction can overlap a shoot action and both
remain independent from locomotion. Each channel snapshot has its own sequence and event ID. Protocol
version 7 removes the old single `state`, input `yaw`, `sprint` boolean, and single `action_layer`
fields without legacy aliases; versions 1–6 are rejected. Snapshots separate owner prediction/debug
state from remote animation/interpolation state and carry gait phase, phase progress, and turn-in-place
direction/progress. Input requests `walk`, `run`, or `sprint`; the server derives actual gait from the
collision-resolved horizontal speed.

## Simulation order

1. Apply sequenced input intent.
2. Derive desired gait, view-relative movement, and desired facing.
3. Apply acceleration/braking or air acceleration, then the rate-limited rotation solver.
4. When grounded on a walkable floor, project desired motion onto the sampled floor plane and
   disable controller gravity; the ground solver owns vertical contact. Jump restores configured
   gravity before the physics step.
5. Submit solved movement to Bullet and step the room physics world once. The Bullet character
   controller's configured native step height resolves server-side stairs; gameplay code does not
   teleport the controller over a detected obstacle.
6. Probe the capsule center and four peripheral points with sphere sweeps. Resolve floor normal,
   signed distance, walkability, slope angle, contact entity, sample count, and short ground grace.
   Small nearby gaps snap to a walkable surface; rises above `character_step_height` do not.
7. Update jump, apex, fall, and landing phases.
8. Resolve attacks/projectiles, damage, health, and death in separate stages.
9. Derive locomotion/action/life channels and publish snapshots.

Grounded movement approaches the desired horizontal velocity with gait-specific piecewise acceleration
curves. Braking combines a normalized piecewise response curve with configured deceleration and
speed-scaled ground friction. Direction changes preserve lateral momentum while directional friction
and turn/pivot deceleration resist abrupt changes. Air steering changes existing horizontal velocity
by a bounded acceleration and never replaces it with a fraction of desired speed. View rotation is
input; character facing is server-solved and rate limited with a configured turn-speed curve.

The authoritative floor query uses five Sphere Sweeps (center/front/back/left/right), not a single
ray. `max_walkable_slope` is shared by the Bullet controller and gameplay classification. Walkable
slope movement is projected onto the sampled ground plane. Ground contact may be held only for the
configured short grace window; grace never validates a jump. The lab arena includes a 20° ramp and a
0.3 m stair run. The browser prediction replica attempts step-up only after a previous grounded move
was clipped, and accepts only a walkable support within the configured rise and current tick's
horizontal displacement. It also checks overhead clearance and overlap before committing. Step-down
uses nearby floor snap. These are lab collision fixtures, not map-authoring facilities. Bullet remains
authoritative; browser collision is a prediction replica rather than a Bullet-equivalent solver.

## Network motion

Each client input carries a monotonic sequence and client tick. Browser input sampling uses a fixed-step
scheduler with a four-input catch-up cap and starts only after the first authoritative owner snapshot,
so the initial server ACK has a local prediction base. The bounded prediction history retains a rolling
state anchor and the state after each input. An authoritative snapshot acknowledges a sequence;
the client restores that server result and deterministically replays later inputs. Stale ACKs are
ignored, future ACKs are rejected, and ACKs whose input state has fallen out of history cause a measured
hard resync that discards the unavailable replay. Position, yaw, and velocity errors and hard-resync
reasons are exposed to the lab. Reconciliation offset is applied to a separate visual transform and
decays over time, with a snap threshold for teleports or large errors. Remote snapshots retain tick,
position, velocity, facing, and yaw rate; render time is delayed and bounded cubic Hermite interpolation
limits visual speed, overshoot, and yaw rate.

The JSON wire model is strict. Protocol version 7 carries `view_yaw`, `view_pitch`, `rotation_mode`, and
`requested_gait`; snapshots report `actual_gait` separately. The JSON tuning contains the solver curves.
Inputs carry intent only; snapshots carry independent movement channels, floor sample, aim offset,
gait phase, five independently revisioned animation channels, and input ACK. Gait phase advances by
collision-resolved travel distance divided by the gait's stride length, and does not advance during
airborne or unsupported ground-grace movement.
Welcome also carries the versioned static collision profile and capsule dimensions. Python builds its
Bullet arena from the same packaged `web/motion/arena-collision.json` profile; per-server arena extents
are applied before the expanded profile is sent. Snapshots include the blocked-move count needed to
replay step-up decisions.

Browser collision prediction sweeps the character center against radius-expanded static AABBs and
ramp side/high-end faces, slides along the first contact, uses sampled plane/box/ramp support for
ground resolution, and performs a bounded step-up only after a previous tick was collision-limited. It
supports short ground grace for movement continuity but still requires confirmed, walkable contact for
jumps. The collision replica does not
reproduce all Bullet capsule/ghost-sweep edge behavior. Golden parity tests cover unconstrained solver
equations; Bullet collision remains covered by separate authoritative-scene tests.

## Browser animation runtime

`MotionFrame` is the only input to the animation-motion modules. It is built from reconciled/local
simulation state, not directly from WebSocket messages. Owner corrections smooth position and yaw,
while remote transforms sample buffered snapshots using estimated server time and adaptive delay.
Animation runtime implementation and synthetic-pose limitations are described in
[`animation-runtime.md`](animation-runtime.md). The 2D canvas client has no production 3D renderer or
authored character animation assets.

## Stages and acceptance

- Phase A: gait-aware acceleration/braking curves, preserved directional momentum, requested versus
  actual gait, and Python/JS movement golden vectors are implemented.
- Phase B: five-point ground probes, configured slope limits, ground contact/grace, walkable slope
  projection, Bullet-native server stairs, bounded replica step-up, ramp obstruction, and step-down
  snapping are implemented and tested against Bullet and browser geometry.
- Phase C: owner prediction history stores solved movement/floor/collision channels with a rolling
  base anchor. ACK reconciliation restores and replays pending inputs through a capsule-radius collision
  replica; stale/future/missing ACK states and history overflow have explicit ignore, reject, or hard
  resync behavior and counters. Correction position/rotation/velocity errors are exposed in the lab.
- Phase D: owner position/yaw visual transforms smooth small corrections and snap large errors;
  remote snapshots use bounded position/yaw Hermite interpolation, capped extrapolation, teleport
  resets, server clock estimation, and adaptive interpolation delay.
- Phase E: blend space selects three local samples across eight circular travel directions, blend
  weights are clamped and normalized, and synthetic skeletons support quaternion pose blending, clip
  sampling, multi-pose blending, world transforms, and transition-triggered pose inertialization.
- Phase F: nine-sample additive aim-pose evaluation, bone-masked upper-body actions, additive hit
  reactions, full-body and life overrides, independent action expiry, distance-driven gait phase, and
  server-replicated turn/phase progress are implemented and tested.
- Phase G: bounded foot correction and world-space foot locks against static collision-profile support
  geometry, distributed pose orientation warp, local root-motion extraction/application, time-windowed
  target-directed root-motion correction, movement-solver trajectory rollout, critical-bone pose
  history/features including contacts, normalized brute-force pose search, and motion matching with
  minimum hold/switch hysteresis are implemented with synthetic-rig and collision-rollout tests. The
  Motion Lab exercises the runtime against its explicitly synthetic seven-bone rig and candidate data.

The movement solver and the browser collision replica have different collision primitives from Bullet;
golden parity covers the shared unconstrained movement equations while collision correction remains
server-authoritative. The browser can now measure prediction, animation, search, and trajectory costs,
but no representative production-server benchmark has been established. Real skeletal playback,
renderer-backed probes, authored asset quality, and two-bone leg IK are not part of this 2D Motion Lab.
