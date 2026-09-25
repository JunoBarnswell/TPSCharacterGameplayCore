# Animation Runtime 3.0 — Continuous Pose Pipeline

## Ownership and final pose path

`CharacterAnimationRuntime.update(MotionFrame, trajectory, dt, probes)` is the Motion Lab's sole Final Pose producer. It is independent of DOM, Canvas, WebSocket and Panda3D. The Canvas skeleton view reads `output.pose`; diagnostics read the same update result.

```text
MotionFrame + predicted trajectory
  → tagged Pose Search and clip-backed MotionMatcher
  → marker mapping / clip playback
  → derivative-aware PoseInertializer
  → additive aim and ordered action/life layers
  → stateful orientation warp and presentation stride scaling
  → per-foot plant state, pelvis stabilization and two-bone leg IK
  → optional bounded root-motion warp preview
  → immutable Final Pose
```

The server owns capsule position, collision, gait semantics and the replicated gait phase. The client owns sampled clip playback, stride presentation, IK and rendered Final Pose. The versioned `web/motion/locomotion-tuning.json` supplies stride lengths, reference speeds and phase markers to Python and JavaScript. Client playback rate is clamped to 0.85–1.15; stride scaling handles the remaining speed difference without modifying capsule movement. Authoritative phase is used in query contact features and marker mapping at clip changes; the clip's own marker phase controls presented foot contact between changes.

The synthetic library creates 11-bone clips once. Motion Matcher plays them via `ClipSampler`, advances time, searches continuing-pose candidates, and jumps only after hold/hysteresis. Locomotion includes idle, eight directions for walk/run/sprint, pivot and turns; airborne and impact clips have their own search tags and phase-specific poses. Database features come from sequential clip samples and include foot velocity. Pose vectors use root-local coordinates. Database construction freezes per-dimension mean and scale before search.

Foot planting has `free → candidate → planting → planted → releasing`, contact thresholds, support-loss grace, capped lock stretch, correction speed, pelvis velocity and foot tilt/slew. IK rotates thigh and calf to reach the target; the synthetic chain's foot child translation stays authored. Motion inertialization uses preceding output and incoming target samples to estimate linear and angular velocity, with bounded transient acceleration. Motion warp has no generated-warp mode: if authored root translation/yaw is zero, correction share is zero.

Remote snapshots are sampled on the delayed render timeline. Continuous values interpolate (gait phase across wrap); movement, life and action revisions advance on that timeline. A newly observed active action enters with its elapsed progress; an already expired action stays inactive. The local fixed scheduler latches brief jump presses, reports discarded stall time and does not introduce extra simulation ticks.

## Evidence and maturity

| Capability | Algorithm | Final Pose path | Synthetic acceptance | Production assets | 3D renderer |
| --- | --- | --- | --- | --- | --- |
| Clip-backed Motion Matching | Yes | Yes | Yes | No | Canvas skeleton only |
| Aim, shoot, hit and death layers | Yes | Yes | Run/aim, shoot/hit and life override integration tests | No | Canvas skeleton only |
| Marker phase, directional movement | Yes | Yes | Marker, eight-direction and switch tests | No | Canvas skeleton only |
| Stride correction and leg IK | Yes | Yes | 3/4.5/6 m/s foot slide and reachable-chain tests | No | Canvas skeleton only |
| Inertialized transitions | Yes | Yes | C0, finite-difference linear/angular velocity and interruption tests | No | Canvas skeleton only |
| Remote render timeline | Yes | Remote sampled skeleton | Snapshot delay and action progress tests | Not applicable | Canvas skeleton |

For the 4.5 m/s synthetic run clip, left-foot world displacement over the first 60 ms of stance at 3.0 m/s was 0.0501 m without stride scaling and 0.0048 m with it. At 6.0 m/s it was 0.0655 m versus 0.0226 m. At authored 4.5 m/s both were 0.0033 m. This measures one synthetic plant window; it is not a production walk-cycle certification.

`tests/test_animation_runtime.mjs` checks clip-driven Final Pose changes, empty groups, 0/45/90/180° search ranking and cost invariance, opposite swing velocities at the same foot position, marker transfers, leg reach, brief support loss, bounded yaw and seam traversal, delayed action starts, zero-motion warp, jump edge buffering, fixed-stall counters and quantitative motion traces. Existing Node, Python and Ruff suites remain part of the gate.

## Remaining work for an Unreal Engine-class result

- Replace procedurally authored clips with retargeted production motion capture and foot contact annotations. Validate phase mapping on real unequal clip cycles and transitions across different rigs.
- Attach Final Pose to a skinned 3D renderer; the current local and remote Canvas skeletons only prove runtime wiring and expose diagnostics.
- Extend contact dynamics for moving platforms, steep terrain, missing probes and production collision queries. Add ground-normal hysteresis and distributed performance benchmarks for crowds.
- Add full scenario traces with thresholds for walls, stairs, ramps, 50/100/150 ms latency, jitter and packet loss. Current quality trace records these fields but synthetic tests cover only a subset of situations.
- Improve same-clip jump selection with full sampled continuing-pose feature costs, and author traversal root motion with explicit caps and predictive target ownership. Do not claim production-complete Motion Matching or traversal from the present synthetic tests.

A PR must remain open until CI, Codex Code Review, Security Review and P1/P2 disposition have finished. This document reports observed maturity; the existence of a tested helper alone does not imply production readiness.
