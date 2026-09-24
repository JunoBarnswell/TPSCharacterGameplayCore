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

## Status

- **Implemented:** local locomotion sample selection, blend-weight safety, skeleton hierarchy, local
  pose transforms, clip sampling, quaternion blend, weighted pose blend, world transform composition,
  synthetic pose inertialization.
- **Experimental:** scalar response curves and current sample layout are code-authored examples, not
  animation content validated by an animator.
- **Reserved:** renderer adapter, asset import pipeline, retargeting, and authored production clips.
- **Not implemented at Phase E:** aim-pose sample composition, bone masks/action-layer blending, real
  foot contact probes, pose-deforming orientation warping, root-motion playback, motion warping,
  trajectory-driven pose search, and motion matching. These remain Phase F/G work until their
  algorithms and tests land.
