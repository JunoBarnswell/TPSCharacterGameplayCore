# Browser Animation Runtime

The browser animation runtime consumes `MotionFrame` and produces poses. It does not read WebSocket
messages, modify authoritative gameplay state, or depend on a particular renderer. The current Motion
Lab runs the runtime with synthetic test skeletons; it does not ship a character rig or production
animation assets.

## Implemented

- `Skeleton` owns an ordered bone hierarchy and local bind transforms.
- `Pose` stores immutable local translation, quaternion rotation, and scale for every bone.
- `worldTransforms` composes local transforms through the hierarchy.
- `AnimationClip`, `AnimationTrack`, `Keyframe`, and `ClipSampler` sample keyed local transforms with
  translation/scale interpolation and shortest-path quaternion slerp.
- `blendPoses` and `blendWeightedPoses` produce local-space poses, normalizing multi-pose weights.
- `PoseInertializer` captures real transform offsets at a transition and decays translation, scale,
  and quaternion rotation offsets with a critically damped envelope.
- `BlendWeightSmoothing` is explicitly weight-only smoothing. Every update clamps and normalizes its
  output; it is not described as pose inertialization.
- The locomotion blend space selects the three nearest local samples and treats its direction axis as
  circular, including continuity across `-180°/+180°`; its fixture covers forward, backward, lateral,
  and diagonal samples for walk, run, and sprint.
- Aim Offset uses a nine-pose grid and bilinear weights. Its output is blended as an additive pose over
  the locomotion pose and can be restricted by a bone mask.
- `CharacterPoseLayerStack` composes independently revisioned locomotion, upper-body action, additive
  reaction, full-body override, and life override channels in a fixed order. Shoot and hit-reaction
  channels can remain active at the same time and expire independently. The pose graph resolves each
  action by channel and state.
- `MotionFrame` carries locomotion phase progress and turn direction, remaining angle, and progress;
  protocol v7 owner and remote snapshot profiles replicate these values together with distance-driven
  gait phase and action-channel revisions.
- `solveFootIK` applies bounded pelvis and full 3D foot corrections against left/right support probes
  sampled from the static floor, box, and ramp collision profile, aligns each foot with the sampled
  surface normal, and accepts persistent world-space foot locks. This is not renderer-backed raycast
  contact.
- `warpPoseOrientation` distributes a bounded yaw correction over configured skeleton bones.
- `extractRootMotionDelta` converts sampled root-pose changes into clip-local translation and yaw;
  `applyRootMotionDelta` applies that delta to a visual transform without changing authoritative state.
- `MotionWarpTarget` and `warpRootMotionDelta` distribute endpoint translation/yaw correction across
  the remaining root-motion clip displacement inside an optional smooth action-time window. A target
  with a window requires an explicit sample time. The lab only activates its target for the synthetic
  `synthetic_vault` channel; no gameplay action or authored vault clip is wired to it.
- `rolloutTrajectory` repeatedly runs the prediction movement and collision solvers for future input
  intent, producing the configured future position, velocity, facing, gait mode, and phase samples.
- `PoseHistory` stores immutable pose references, critical-bone world positions and rotations, root
  velocity, contact labels, and extracted features. `PoseDatabase`, `PoseSearch`, and `MotionMatcher`
  execute a weighted brute-force search over pose, trajectory, velocity, facing, contact, and
  continuity costs. Cost groups are normalized from database variance with positive scale floors;
  minimum clip hold, same-clip phase continuity, and switch hysteresis reduce candidate thrashing.
- The Motion Lab runs those algorithms against a seven-bone synthetic rig, static collision-profile
  support probes, and a generated synthetic pose database. It reports selected clip/pose, candidate
  costs, IK/warp/search timings, trajectory samples, and root offsets. This is an algorithm diagnostic,
  not a renderer.

These are data and pose algorithms. Their tests use small synthetic skeletons and clips. They do not
establish GPU skinning, renderer compatibility, asset import, authoring workflow, or final character
quality.

## Runtime boundaries

```text
MotionFrame
  -> locomotion weights
  -> clip sampling / pose blending
  -> additive or masked layers
  -> pose-space warping and IK
  -> renderer adapter
```

Python only supplies authoritative motion semantics and replicated floor data. Browser systems own
clip sampling, pose composition, visual correction, IK, and presentation. A future renderer adapter
must preserve the `Skeleton`/`Pose` contract or explicitly convert it to the renderer's skeleton type.

## Phase G status

- **Implemented:** eight-direction gait blend space, blend-weight safety, skeleton/pose/clip sampling,
  quaternion and multi-pose blending, transform-level pose inertialization, additive aim, independent
  animation channels, direct foot-bone IK using collision-profile supports, world-space foot locks,
  distributed skeleton yaw warp, visual root offset, root-motion extraction/application, time-windowed
  target correction, solver-rollout trajectory, contact-aware critical pose history/features, normalized
  brute-force pose search, and motion matching with transition hysteresis.
- **Experimental:** current synthetic seven-bone rig, authored response/sample curves, and direct foot
  bone correction are algorithm fixtures; they have not been evaluated on production skeletons or
  animator-authored clips. Foot IK does not solve a knee chain.
- **Reserved:** production renderer adapter, real raycast/renderer contact probes, asset import and
  retargeting, animation authoring tools, large-database search acceleration, and production clips.
- **Not implemented:** GPU skinning, production asset playback, animator-authored Motion Matching
  database, full two-bone IK, contact-aware root motion extraction from compressed assets, and a
  renderer-integrated traversal pipeline.
