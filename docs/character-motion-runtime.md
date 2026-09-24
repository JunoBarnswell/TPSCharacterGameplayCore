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
gait, facing/view rotations, rotation mode, and locomotion phase. `Character` separately owns
`ActionLayer` and `LifeState`. A hit reaction therefore overlays locomotion instead of replacing it.
The protocol is version 4. It removes the old single `state`, input `yaw`, and `sprint` boolean fields
without legacy aliases, and rejects protocol versions 1–3. Input requests `walk`, `run`, or `sprint`;
the server independently derives actual gait from solved horizontal speed.

## Simulation order

1. Apply sequenced input intent.
2. Derive desired gait, view-relative movement, and desired facing.
3. Apply acceleration/braking or air acceleration, then the rate-limited rotation solver.
4. When grounded on a walkable floor, project desired motion onto the sampled floor plane and
   disable controller gravity; the ground solver owns vertical contact. Jump restores configured
   gravity before the physics step.
5. Submit solved movement to Bullet and step the room physics world once. A clipped grounded move is
   compared against its requested velocity; only a detected block can trigger step-up assistance.
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
0.3 m stair run. Step-up is attempted only after a grounded move was clipped; the solver validates
support height and walkable normal before advancing onto it. Step-down uses nearby floor snap. These
are lab collision fixtures, not map-authoring facilities. Bullet remains authoritative; browser
collision is a prediction replica rather than a Bullet-equivalent solver.

## Network motion

Each client input carries a monotonic sequence and client tick. The owner prediction history stores
the state immediately after each local input. An authoritative snapshot acknowledges a sequence;
the client restores its movement state at the server result and deterministically replays later
inputs. Reconciliation offset is applied to a separate visual position and decays over time, with a
snap threshold for teleports or large errors. Remote snapshots retain tick, position, velocity, and
rotation; render time is delayed and cubic Hermite interpolation uses endpoint velocities.

The JSON wire model is strict. Protocol version 4 carries `view_yaw`, `view_pitch`, `rotation_mode`, and
`requested_gait`; snapshots report `actual_gait` separately. The JSON tuning contains the solver curves.
Inputs carry intent only; snapshots carry independent movement channels, floor sample, aim offset,
and input ACK.
Welcome also carries the versioned static collision profile and capsule dimensions. Python builds its
Bullet arena from the same packaged `web/motion/arena-collision.json` profile; per-server arena extents
are applied before the expanded profile is sent. Snapshots include the blocked-move count needed to
replay step-up decisions.

Browser collision prediction sweeps the character center against radius-expanded static AABBs, slides
along the first contact, uses sampled plane/box/ramp support for ground resolution, and performs a
bounded step-up only after a previous tick was collision-limited. The collision replica does not
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
  projection, blocked-move step-up, and step-down snapping are implemented and tested against Bullet.
- Phase C: owner prediction history stores solved movement/floor/collision channels, ACK reconciliation
  restores and replays pending inputs through a capsule-radius collision replica, and correction
  position/rotation/velocity errors plus large-correction counts are exposed in the lab.
- Phase D: owner position/yaw visual transforms smooth small corrections and snap large errors;
  remote snapshots use bounded Hermite interpolation, capped extrapolation, teleport resets, server
  clock estimation, and adaptive interpolation delay.
- Phase E: blend space selects three local samples with a circular direction axis, blend weights are
  clamped and normalized, and synthetic skeletons support quaternion pose blending, clip sampling,
  multi-pose blending, world transforms, and transform-level pose inertialization.
- Phase F planned: additive aim-pose sampling, animation action-layer stack, bone masks, and transition
  progress driven by `MotionFrame`.
- Phase G planned: tested foot IK, pose-deforming orientation warp, root-motion/motion-warp transforms,
  solver-rollout trajectories, pose features/history, pose search, and motion matching.

Real skeletal playback, renderer-backed foot probes, and asset-authored quality are not part of the
2D Motion Lab even after the synthetic pose runtime is complete.
