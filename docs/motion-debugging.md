# Character Motion Lab Debugging

Open `/` while the Python server is running. The 2D arena continues to show server-authoritative
players, predicted owner movement, projectiles, and static collision fixtures. The stats panel is an
engine diagnostic surface and labels synthetic animation data explicitly.

## Movement and network

The panel shows simulation/snapshot/render rates; position, velocity, acceleration, desired velocity;
requested and actual gait; movement mode and phase progress; ground probe count, floor normal, distance
and slope; character/view/aim yaw and pitch; input ACK and pending input count; position/rotation/
velocity reconciliation errors; RTT/jitter; adaptive interpolation delay; server-clock phase; and
remote buffer size/extrapolation. Server telemetry also shows the minimum observed room tick rate in a
one-second window, latest and p95 full-tick duration, movement/physics/snapshot p95 durations, largest
and average per-recipient snapshot payload, and enqueued snapshot bytes/messages per second. These
counts measure accepted outbound queue entries; the session queue may coalesce snapshots before a
WebSocket send. Full tick duration includes world simulation, event dispatch, and snapshot
validation/serialization/enqueue.

The latency, jitter, and packet-loss sliders delay/drop application messages for local evaluation. They
do not emulate kernel/network queue behavior or loss of the underlying WebSocket connection.

## Animation motion diagnostics

The page runs trajectory prediction through the collision-aware solver and shows the five future
samples and rollout time. A seven-bone synthetic rig runs distributed orientation warp, bounded foot
IK and world-space locks against static floor/box/ramp support probes, visual root offset, local
root-motion extraction, and a target-warp preview limited to the synthetic vault action window. The
live Pose History shows bounded sample count and critical bones.

The synthetic Pose Database generates idle, eight-direction walk/run/sprint, pivot, and turn samples
at gait phases with contact labels. The page reports current semantic candidate, selected
clip/time/pose ID, candidate count, total and component costs, transition reason, and search time.
ACK history overflow and hard-resync reason/count are visible alongside nullable reconciliation
errors. IK, warp, search, graph, and trajectory timings use `performance.now()` and are diagnostic
samples, not benchmark percentiles.

## Production boundary

The lab does not contain a WebGL renderer, skinning, authored animation clips, renderer raycasts, or a
production character rig. Its collision-profile foot supports and synthetic pose database validate
algorithms and observability only. Python phase timings and full-test results are separate server-side
evidence. Performance
percentiles should be collected with a reproducible player/room workload before claiming a production
capacity target. The checked-in tests verify a 16-player room's fixed-tick rate; the local 16-WebSocket
load run is a development-machine sample, not a production-host capacity benchmark.
