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
  circular, including continuity across `-180°/+180°`.
- Aim Offset uses a nine-pose grid and bilinear weights. Its output is blended as an additive pose over
  the locomotion pose and can be restricted by a bone mask.
- `CharacterPoseLayerStack` composes upper-body override, additive reaction, full-body override, and
  life override channels in a fixed order. The pose graph can resolve simultaneous `shoot` and
  `hit_reaction` entries from `MotionFrame.actionLayers`.
- `MotionFrame` carries locomotion phase progress and turn direction, remaining angle, and progress;
  protocol v6 owner and remote snapshot profiles replicate these values.
- `solveFootIK` applies bounded pelvis and full 3D foot corrections against left/right world probes,
  aligns each foot's up axis with its sampled normal, and accepts persistent world-space foot locks.
- `warpPoseOrientation` distributes a bounded yaw correction over configured skeleton bones.
- `extractRootMotionDelta` converts sampled root-pose changes into clip-local translation and yaw;
  `applyRootMotionDelta` applies that delta to a visual transform without changing authoritative state.
- `MotionWarpTarget` and `warpRootMotionDelta` distribute endpoint translation/yaw correction across
  the remaining root-motion clip displacement.
- `rolloutTrajectory` repeatedly runs the prediction movement and collision solvers for future input
  intent, producing the configured future position, velocity, facing, gait mode, and phase samples.
- `PoseHistory` stores immutable pose references, critical-bone world positions and rotations, root
  velocity, and extracted features. `PoseDatabase`, `PoseSearch`, and `MotionMatcher` execute a
  weighted brute-force search over pose, trajectory, velocity, facing, and continuity costs.
- The Motion Lab runs those algorithms against a seven-bone synthetic rig, synthetic contact probes,
  and a small synthetic pose database. It reports selected clip/pose, candidate costs, IK/warp/search
  timings, trajectory samples, and root offsets. This is an algorithm diagnostic, not a renderer.

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

- **Implemented:** blend space, blend-weight safety, skeleton/pose/clip sampling, quaternion and
  multi-pose blending, transform-level pose inertialization, additive aim, masked action layers,
  direct foot-bone IK with lock targets, distributed skeleton yaw warp, visual root offset, root-motion
  extraction/application, target-directed root-motion correction, solver-rollout trajectory, critical
  pose history/features, brute-force pose search, and motion matching.
- **Experimental:** current synthetic seven-bone rig, authored response/sample curves, and direct foot
  bone correction are algorithm fixtures; they have not been evaluated on production skeletons or
  animator-authored clips. Foot IK does not solve a knee chain.
- **Reserved:** production renderer adapter, real raycast/renderer contact probes, asset import and
  retargeting, animation authoring tools, large-database search acceleration, and production clips.
- **Not implemented:** GPU skinning, production asset playback, animator-authored Motion Matching
  database, full two-bone IK, contact-aware root motion extraction from compressed assets, and a
  renderer-integrated traversal pipeline.
