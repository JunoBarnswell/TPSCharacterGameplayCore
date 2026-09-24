# Trajectory and Motion Matching

## Trajectory prediction

`rolloutTrajectory` clones the current movement state and advances it through the browser prediction
movement solver at fixed steps. Each step consumes either one fixed intent or an intent callback that
can change over the forecast horizon. The optional collision world runs the same capsule-radius
prediction collision, wall slide, step, and ground solve as normal input replay. Samples report
requested future time, actual sample time, position, velocity, facing, speed, movement mode, and
locomotion phase. The Motion Lab updates this rollout from current movement input at a throttled rate.

## Pose history and features

`PoseHistory` retains sampled immutable `Pose` values with tick and clip metadata, root velocity, and
world positions/rotations for configured critical bones (the default is root, pelvis, and both feet).
Features include root velocity/facing; pelvis and foot positions relative to root; pelvis and foot
velocities from consecutive samples; and relative future trajectory position, facing, and velocity.
History capacity is bounded and ticks must increase monotonically.

## Database, search, and transition

`PoseDatabase` validates stable IDs and a fixed feature-vector schema. `PoseSearch` currently performs
a deterministic brute-force scan and ranks candidates by weighted RMS costs for pose shape, trajectory,
velocity, facing, and same-pose/same-clip continuity. `MotionMatcher` supplies current pose features,
retains the current candidate for continuity scoring, and reports the selected pose, clip/time,
candidate count, cost breakdown, and transition reason.

This is an implemented search algorithm, not an asset-backed production matcher. The lab database
contains seven labeled synthetic poses on a seven-bone rig. The fixture proves that forward, idle,
turn, and reverse intent can select different candidates from actual pose/trajectory costs. Large
database indexing, feature normalization from authored datasets, annotation tools, clip playback
transitions, and animator review are reserved work.

## Root motion and target warping

`extractRootMotionDelta` measures sampled root transforms and converts translation into the previous
root's local orientation. `applyRootMotionDelta` maps that delta through visual character facing and
does not mutate the authoritative transform. `MotionWarpTarget` names an immutable target transform;
`warpRootMotionDelta` proportionally allocates translation and yaw endpoint error across the current
and remaining clip displacement. Tests exercise a synthetic traversal sequence that reaches its
target. No production traversal clip or Bullet-authoritative root-motion action is supplied.

## Limitations

- Brute force is intentionally simple and is suitable only for a small candidate set.
- Pose costs use unnormalized physical feature dimensions; production datasets need authored scales,
  contact labels, and asset validation.
- The current runtime does not search compressed production animation assets, serialize databases,
  or expose a GPU/renderer pose adapter.
- Root motion is visual/runtime data. Authoritative gameplay movement still uses the capsule solver.
