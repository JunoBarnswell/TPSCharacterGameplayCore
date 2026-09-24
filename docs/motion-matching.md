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

`PoseDatabase` validates stable IDs, contact labels, optional clip duration, and a fixed feature-vector
schema. `PoseSearch` performs a deterministic brute-force scan and ranks candidates by weighted RMS
costs for pose shape, trajectory, velocity, facing, contacts, and same-pose/same-clip continuity. Each
feature group is normalized by database variance with a positive scale floor so world-position,
velocity, and binary contact magnitudes do not dominate solely because of units. `MotionMatcher`
retains the active candidate for continuity scoring and reports the selected pose, clip/time, candidate
count, cost breakdown, and transition reason. A minimum clip hold, same-clip phase continuity, and
switch-cost hysteresis limit rapid clip switching; playback time wraps by clip duration when supplied.

This is an implemented search algorithm, not an asset-backed production matcher. The lab database is
generated from a seven-bone synthetic rig with idle, eight-direction walk/run/sprint, pivot, and turn
samples at two gait phases, including contact labels. The fixture proves that forward, idle, turn, and
reverse intent can select different candidates from pose, trajectory, and contact costs. Large-database
indexing, authored-dataset feature validation/scales, annotation tools, actual production clip playback,
and animator review are reserved work. The lab renderer uses the selected synthetic clip label to
choose a generated procedural pose target; it does not play an imported animation asset.

## Root motion and target warping

`extractRootMotionDelta` measures sampled root transforms and converts translation into the previous
root's local orientation. `applyRootMotionDelta` maps that delta through visual character facing and
does not mutate the authoritative transform. `MotionWarpTarget` names an immutable target transform;
`warpRootMotionDelta` proportionally allocates translation and yaw endpoint error across the current
and remaining clip displacement inside a smooth action-time window. A windowed target requires a sample
time; outside the window the warp weight eases to zero. Tests exercise a synthetic traversal sequence
that reaches its target. The browser preview only activates for a synthetic vault channel. No production
traversal clip or Bullet-authoritative root-motion action is supplied.

## Limitations

- Brute force is intentionally simple and is suitable only for a small candidate set.
- Production databases still need authored feature validation, reliable contact labels, and asset
  validation; fixture-derived normalization is not evidence of production feature quality.
- The current runtime does not search compressed production animation assets, serialize databases,
  or expose a GPU/renderer pose adapter.
- Root motion is visual/runtime data. Authoritative gameplay movement still uses the capsule solver.
