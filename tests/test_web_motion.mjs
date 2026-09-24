import test from "node:test";
import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

import {
  angleDelta,
  deriveActualGait,
  evaluateResponseCurve,
  desiredMotion,
  predictMovementStep,
  projectVelocityOntoGroundPlane,
  solveHorizontalVelocity,
  solveRotation,
} from "../aster_game/web/motion/movement-solver.mjs";
import {
  PredictionHistory,
  FixedStepScheduler,
  RemoteSnapshotBuffer,
  VisualTransformSmoothing,
  AdaptiveInterpolationDelay,
  ServerClockEstimator,
} from "../aster_game/web/motion/network-motion.mjs";
import {
  createMotionFrame,
  MotionHistory,
} from "../aster_game/web/motion/motion-frame.mjs";
import {
  evaluateAnimationGraph,
  evaluatePoseAnimationGraph,
} from "../aster_game/web/animation/animation-graph.mjs";
import { BlendWeightSmoothing } from "../aster_game/web/animation/blend-weight-smoothing.mjs";
import { evaluateBlendSpace } from "../aster_game/web/animation/blend-space.mjs";
import { evaluateAimOffset } from "../aster_game/web/animation/aim-offset.mjs";
import { orientationWarpAngle, warpPoseOrientation } from "../aster_game/web/animation/orientation-warp.mjs";
import { solveFootIK, FootLockState } from "../aster_game/web/animation/foot-ik.mjs";
import { applyVisualRootOffset } from "../aster_game/web/animation/root-offset.mjs";
import { applyRootMotionDelta, extractRootMotionDelta } from "../aster_game/web/animation/root-motion.mjs";
import {
  MotionWarpTarget,
  MotionWarpWindow,
  warpRootMotionDelta,
} from "../aster_game/web/animation/motion-warp.mjs";
import { PoseHistory } from "../aster_game/web/animation/motion-matching/pose-history.mjs";
import { PoseDatabase } from "../aster_game/web/animation/motion-matching/pose-database.mjs";
import { PoseSearch } from "../aster_game/web/animation/motion-matching/pose-search.mjs";
import { MotionMatcher } from "../aster_game/web/animation/motion-matching/motion-matcher.mjs";
import { rolloutTrajectory } from "../aster_game/web/motion/trajectory.mjs";
import {
  AnimationClip,
  AnimationTrack,
  Keyframe,
  ClipSampler,
  sampleAnimationClip,
} from "../aster_game/web/animation/clip.mjs";
import {
  blendPoses,
  blendWeightedPoses,
  createTransform,
  createSkeleton,
  createPose,
  Pose,
  quaternionSlerp,
  rotateVector,
  worldTransforms,
} from "../aster_game/web/animation/pose.mjs";
import { PoseInertializer } from "../aster_game/web/animation/pose-inertializer.mjs";
import {
  AnimationLayerChannel,
  BoneMask,
  CharacterPoseLayerStack,
} from "../aster_game/web/animation/layers.mjs";
import { evaluateAimOffsetPose } from "../aster_game/web/animation/aim-offset.mjs";
import { CollisionWorld } from "../aster_game/web/motion/collision-world.mjs";
import { ActionChannelSnapshotCursor } from "../aster_game/web/animation/action-channel-state.mjs";

const dt = 1 / 60;
const goldenVectors = JSON.parse(readFileSync(
  new URL("./fixtures/movement-golden-vectors.json", import.meta.url),
  "utf8",
));
const tuning = {
  walk_speed: 2,
  run_speed: 4.5,
  sprint_speed: 6.5,
  ground_directional_friction: 9,
  turning_deceleration: 7,
  pivot_braking_multiplier: 1.75,
  walk_acceleration_curve: [[0, 1.25], [0.5, 1], [1, 0.6]],
  run_acceleration_curve: [[0, 1.35], [0.5, 1], [1, 0.65]],
  sprint_acceleration_curve: [[0, 1.5], [0.5, 1], [1, 0.7]],
  braking_curve: [[0, 0.6], [0.35, 1], [1, 1.35]],
  turn_speed_curve: [[0, 0.22], [0.25, 0.55], [1, 1]],
  air_control: 0.45,
  ground_acceleration: 24,
  braking_deceleration: 16,
  ground_friction: 8,
  air_acceleration: 10,
  air_max_speed: 6.5,
  max_rotation_speed: 540,
  rotation_acceleration: 1440,
  rotation_deceleration: 1800,
  turn_in_place_threshold: 45,
  pivot_angle_threshold: 135,
  jump_speed: 6,
  gravity: 9.81,
  max_fall_speed: 50,
  apex_velocity_threshold: 0.15,
  jump_cooldown_seconds: 0.25,
  landing_soft_velocity: 4,
  landing_heavy_velocity: 9,
  landing_recovery_seconds: 0.35,
};
const collisionTuning = {
  ...tuning,
  character_radius: 0.45,
  character_cylinder_height: 0.9,
  character_step_height: 0.35,
  ground_probe_radius: 0.06,
  ground_probe_depth: 0.4,
  ground_probe_start_offset: 0.12,
  ground_snap_distance: 0.35,
  ground_grace_distance: 0.12,
  ground_grace_ticks: 2,
  max_walkable_slope: 50,
};

function idleState() {
  return {
    position: [0, 0, 0],
    velocity: [0, 0, 0],
    acceleration: [0, 0, 0],
    desired_velocity: [0, 0, 0],
    current_speed: 0,
    horizontal_speed: 0,
    vertical_speed: 0,
    gait_phase: 0,
    grounded: true,
    walkable_floor: true,
    ground_contact_confirmed: true,
    ground_contact_point: [0, 0, 0],
    last_grounded_tick: 0,
    floor_normal: [0, 1, 0],
    floor_distance: 0,
    movement_mode: "grounded",
    actual_gait: "idle",
    requested_gait: "run",
    character_yaw: 0,
    desired_facing_yaw: 0,
    angular_velocity: 0,
    view_yaw: 0,
    view_pitch: 0,
    aim_yaw: 0,
    aim_pitch: 0,
    rotation_mode: "orient_to_movement",
    locomotion_phase: "idle",
    life_state: "alive",
    jump_held: false,
    blocked_move_ticks: 0,
  };
}

function channelSnapshot(state = "none", active = false, sequence = 0) {
  return {
    state,
    active,
    start_tick: sequence,
    end_tick: active ? null : sequence,
    sequence,
    event_id: sequence === 0 ? "" : `test:${sequence}`,
    blend_semantic: active ? "masked_override" : "none",
  };
}

function input(sequence, moveZ = 1) {
  return {
    type: "input",
    sequence,
    client_tick: sequence,
    move_x: 0,
    move_z: moveZ,
    jump: false,
    requested_gait: "run",
    view_yaw: 0,
    view_pitch: 0,
    rotation_mode: "orient_to_movement",
    sentAt: sequence * 10,
  };
}

function createMotionRig() {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "pelvis", parentIndex: 0, bindLocal: createTransform([0, 1, 0]) },
    { name: "spine", parentIndex: 1, bindLocal: createTransform([0, 0.3, 0]) },
    { name: "left_foot", parentIndex: 1, bindLocal: createTransform([-0.2, -1, 0]) },
    { name: "right_foot", parentIndex: 1, bindLocal: createTransform([0.2, -1, 0]) },
    { name: "chest", parentIndex: 2, bindLocal: createTransform([0, 0.3, 0]) },
  ]);
  const makePose = ({
    rootPosition = [0, 0, 0],
    rootYaw = 0,
    pelvis = [0, 1, 0],
    leftFoot = [-0.2, -1, 0],
    rightFoot = [0.2, -1, 0],
    chest = [0, 0.6, 0],
  } = {}) => {
    const yaw = rootYaw * Math.PI / 360;
    return createPose(skeleton, [
      createTransform(rootPosition, [0, Math.sin(yaw), 0, Math.cos(yaw)]),
      createTransform(pelvis),
      createTransform([0, 0.3, 0]),
      createTransform(leftFoot),
      createTransform(rightFoot),
      createTransform(chest.map((value, axis) => axis === 1 ? value - 0.3 : value)),
    ]);
  };
  return { skeleton, makePose };
}

test("movement acceleration and braking match the authoritative solver", () => {
  const first = solveHorizontalVelocity([0, 0, 0], [0, 0, 6.5], dt, true, tuning);
  assert.ok(first.velocity[2] > 0 && first.velocity[2] < tuning.sprint_speed);
  const braking = solveHorizontalVelocity(first.velocity, [0, 0, 0], dt, true, tuning);
  assert.ok(braking.velocity[2] < first.velocity[2]);
  assert.ok(braking.velocity[2] > 0);
});

test("JavaScript solver matches the shared Python movement golden vectors", () => {
  const goldenTuning = goldenVectors.tuning;
  for (const sample of goldenVectors.cases) {
    const desired = [sample.desired[0], 0, sample.desired[1]];
    const current = [sample.current[0], 0, sample.current[1]];
    const result = solveHorizontalVelocity(
      current,
      desired,
      goldenVectors.dt,
      sample.grounded,
      goldenTuning,
      sample.requested_gait,
    );
    const expected = sample.expected;
    assertVector(
      [result.velocity[0], result.velocity[2]],
      expected.velocity,
      sample.name,
    );
    assertVector(
      [result.acceleration[0], result.acceleration[2]],
      expected.acceleration,
      sample.name,
    );
    assert.equal(
      deriveActualGait(Math.hypot(result.velocity[0], result.velocity[2]), goldenTuning),
      expected.actual_gait,
      sample.name,
    );
    assert.equal(sample.grounded ? "grounded" : "airborne", expected.movement_mode);
  }
  const rotation = goldenVectors.rotation;
  const solvedRotation = solveRotation(
    rotation.current_yaw,
    rotation.angular_velocity,
    rotation.desired_yaw,
    goldenVectors.dt,
    goldenTuning,
  );
  assert.ok(Math.abs(solvedRotation.yaw - rotation.expected_yaw) < 1e-10);
  assert.ok(Math.abs(solvedRotation.angularVelocity - rotation.expected_angular_velocity) < 1e-10);
});

test("shared solver remains within drift tolerance over 10, 30, and 60 second motion sequences", () => {
  const scenario = JSON.parse(readFileSync(
    new URL("./fixtures/movement-drift-vectors.json", import.meta.url),
    "utf8",
  ));
  const durations = new Set(scenario.durations_seconds);
  const velocity = [0, 0, 0];
  const position = [0, 0, 0];
  let acceleration = [0, 0, 0];
  let characterYaw = 0;
  let angularVelocity = 0;
  const actual = {};
  const maximumTicks = Math.max(...scenario.durations_seconds) / scenario.dt;
  for (let tick = 1; tick <= maximumTicks; tick++) {
    const segmentIndex = Math.floor((tick - 1) / scenario.phase_ticks) % scenario.segments.length;
    const segment = scenario.segments[segmentIndex];
    const desired = desiredMotion(segment, goldenVectors.tuning);
    const solved = solveHorizontalVelocity(
      velocity,
      desired.velocity,
      scenario.dt,
      true,
      goldenVectors.tuning,
      segment.requested_gait,
    );
    velocity.splice(0, 3, ...solved.velocity);
    acceleration = solved.acceleration;
    position[0] += velocity[0] * scenario.dt;
    position[2] += velocity[2] * scenario.dt;
    let desiredYaw = characterYaw;
    if (segment.rotation_mode === "aim" || segment.rotation_mode === "strafe") {
      desiredYaw = segment.view_yaw;
    } else if (Math.hypot(segment.move_x, segment.move_z) > 1e-4) {
      desiredYaw = Math.atan2(desired.velocity[0], desired.velocity[2]) * 180 / Math.PI;
    }
    const rotation = solveRotation(
      characterYaw,
      angularVelocity,
      desiredYaw,
      scenario.dt,
      goldenVectors.tuning,
    );
    characterYaw = rotation.yaw;
    angularVelocity = rotation.angularVelocity;
    const seconds = tick * scenario.dt;
    if (!durations.has(seconds)) continue;
    actual[seconds] = {
      position: [position[0], position[2]],
      velocity: [velocity[0], velocity[2]],
      acceleration: [acceleration[0], acceleration[2]],
      yaw: characterYaw,
      yaw_rate: angularVelocity,
      actual_gait: deriveActualGait(Math.hypot(velocity[0], velocity[2]), goldenVectors.tuning),
    };
  }
  assert.deepEqual(Object.keys(actual).map(Number), scenario.durations_seconds);
  for (const seconds of scenario.durations_seconds) {
    const expected = scenario.expected[String(seconds)];
    for (const field of ["position", "velocity", "acceleration"]) {
      assertVector(actual[seconds][field], expected[field], `${seconds}s ${field}`, scenario.tolerance);
    }
    assert.ok(Math.abs(actual[seconds].yaw - expected.yaw) <= scenario.tolerance, `${seconds}s yaw`);
    assert.ok(Math.abs(actual[seconds].yaw_rate - expected.yaw_rate) <= scenario.tolerance, `${seconds}s yaw rate`);
    assert.equal(actual[seconds].actual_gait, expected.actual_gait, `${seconds}s gait`);
  }
});

function assertVector(actual, expected, label, tolerance = 1e-9) {
  assert.equal(actual.length, expected.length, label);
  for (let index = 0; index < expected.length; index++) {
    assert.ok(Math.abs(actual[index] - expected[index]) <= tolerance, `${label}[${index}]`);
  }
}

test("requested sprint remains distinct while actual gait ramps through speed bands", () => {
  const requested = desiredMotion({ ...input(1), requested_gait: "sprint" }, tuning);
  assert.equal(requested.requestedGait, "sprint");
  assert.equal(Math.hypot(requested.velocity[0], requested.velocity[2]), tuning.sprint_speed);

  let state = { ...idleState(), simulation_tick: 1 };
  state = predictMovementStep(state, { ...input(1), requested_gait: "sprint" }, tuning, dt);
  assert.equal(state.requested_gait, "sprint");
  assert.equal(state.actual_gait, "walk");
  for (let sequence = 2; sequence <= 180; sequence++) {
    state = predictMovementStep(
      state,
      { ...input(sequence), requested_gait: "sprint" },
      tuning,
      dt,
    );
  }
  assert.equal(state.actual_gait, "sprint");
  assert.equal(deriveActualGait(0, tuning), "idle");
});

test("prediction advances gait phase by confirmed grounded distance and preserves it in air", () => {
  let state = {
    ...idleState(),
    horizontal_speed: 2,
    velocity: [0, 0, 2],
    gait_phase: 0.98,
  };
  const grounded = predictMovementStep(state, input(1), tuning, dt);
  assert.ok(grounded.gait_phase >= 0 && grounded.gait_phase < 1);
  assert.ok(grounded.gait_phase < state.gait_phase || grounded.gait_phase > 0.98);
  state = {
    ...grounded,
    grounded: false,
    ground_contact_confirmed: false,
    movement_mode: "airborne",
    velocity: [0, 0, 2],
  };
  const airborne = predictMovementStep(state, input(2), tuning, dt);
  assert.equal(airborne.gait_phase, state.gait_phase);
});

test("direction changes retain momentum and 180 degree pivots brake harder", () => {
  const quarterTurn = solveHorizontalVelocity([0, 0, 6.5], [6.5, 0, 0], dt, true, tuning, "run");
  const pivot = solveHorizontalVelocity([0, 0, 6.5], [0, 0, -6.5], dt, true, tuning, "run");
  assert.ok(quarterTurn.velocity[2] > 6);
  assert.ok(quarterTurn.velocity[0] > 0);
  assert.ok(pivot.velocity[2] > 0);
  assert.ok(Math.hypot(...pivot.velocity) < Math.hypot(...quarterTurn.velocity));
});

test("ground plane projection preserves speed and removes normal velocity", () => {
  const angle = Math.PI / 6;
  const normal = [0, Math.cos(angle), -Math.sin(angle)];
  const projected = projectVelocityOntoGroundPlane([0, 0, 4], normal);
  assert.ok(Math.abs(Math.hypot(...projected) - 4) < 1e-10);
  assert.ok(Math.abs(projected.reduce((sum, value, axis) => sum + value * normal[axis], 0)) < 1e-10);
  assert.ok(projected[1] > 0);
});

test("air steering keeps inherited velocity and applies bounded acceleration", () => {
  const result = solveHorizontalVelocity([4, 0, 0], [0, 0, 5], dt, false, tuning);
  assert.ok(result.velocity[0] > 3.9);
  assert.ok(result.velocity[2] > 0 && result.velocity[2] < 0.1);
});

test("rotation follows the shortest arc and caps angular rate", () => {
  assert.equal(angleDelta(-179, 179), 2);
  const result = solveRotation(0, 0, 90, dt, tuning);
  assert.ok(Math.abs(result.angularVelocity) <= tuning.rotation_acceleration * dt);
  assert.ok(result.yaw > 0 && result.yaw < 1);
});

test("prediction validates jump edges, cooldown, and idle turn-in-place", () => {
  const state = { ...idleState(), simulation_tick: 20, jump_available_tick: 0 };
  const jump = { ...input(1, 0), jump: true };
  const airborne = predictMovementStep(state, jump, tuning, dt);
  assert.equal(airborne.movement_mode, "airborne");
  assert.ok(airborne.vertical_speed > 0);
  const continued = predictMovementStep(airborne, jump, tuning, dt);
  assert.ok(continued.vertical_speed < airborne.vertical_speed);

  const turn = predictMovementStep(
    { ...idleState(), simulation_tick: 20 },
    { ...input(1, 0), view_yaw: 90 },
    tuning,
    dt,
  );
  assert.ok(turn.character_yaw > 0);
});

test("predicted locomotion derives start, loop, pivot, stop and turn phases", () => {
  let state = { ...idleState(), simulation_tick: 1, jump_available_tick: 0 };
  for (let sequence = 1; sequence <= 24; sequence++) {
    state = predictMovementStep(state, input(sequence, 1), tuning, dt);
  }
  assert.equal(state.locomotion_phase, "loop");
  state = predictMovementStep(state, input(25, -1), tuning, dt);
  assert.equal(state.locomotion_phase, "pivot");
  for (let sequence = 26; sequence < 120; sequence++) {
    state = predictMovementStep(state, input(sequence, 0), tuning, dt);
  }
  assert.equal(state.locomotion_phase, "idle");
  state = predictMovementStep(state, { ...input(120, 0), view_yaw: 90 }, tuning, dt);
  assert.equal(state.locomotion_phase, "turn_in_place");
});

test("predicted pivot angle follows the server movement tuning", () => {
  const stateAtSpeed = () => {
    let state = { ...idleState(), simulation_tick: 1, jump_available_tick: 0 };
    for (let sequence = 1; sequence <= 24; sequence++) {
      state = predictMovementStep(state, input(sequence, 1), tuning, dt);
    }
    return state;
  };
  const reverse120 = {
    ...input(25, -0.5),
    move_x: Math.sqrt(0.75),
  };
  const noPivot = predictMovementStep(
    stateAtSpeed(),
    reverse120,
    { ...tuning, pivot_angle_threshold: 130 },
    dt,
  );
  const pivot = predictMovementStep(
    stateAtSpeed(),
    reverse120,
    { ...tuning, pivot_angle_threshold: 110 },
    dt,
  );
  assert.equal(noPivot.locomotion_phase, "loop");
  assert.equal(pivot.locomotion_phase, "pivot");
});

test("owner prediction restores an ACK and replays only remaining input history", () => {
  const base = idleState();
  const commands = [input(1), input(2), input(3)];
  const expected = commands.reduce((state, command) => predictMovementStep(state, command, tuning, dt), base);
  const serverStateAtAck = predictMovementStep(base, commands[0], tuning, dt);
  const history = new PredictionHistory(256, 0.001);
  history.reset(base);
  for (const command of commands) history.predict(command, tuning, dt);
  const corrected = history.reconcile(serverStateAtAck, 1, tuning, dt);
  assert.deepEqual(corrected.position, expected.position);
  assert.deepEqual(corrected.velocity, expected.velocity);
  assert.deepEqual(history.inputs.map(({ input: pending }) => pending.sequence), [2, 3]);
  assert.equal(history.metrics.reconciliation_count, 1);
  assert.equal(history.metrics.position_error, 0);
  assert.equal(history.metrics.velocity_error, 0);
  assert.equal(history.metrics.correction_position_error, 0);
  assert.equal(history.metrics.large_correction_count, 0, JSON.stringify(history.metrics));
});

test("prediction history reports overflow and hard-resyncs when an ACK predates retained input", () => {
  const history = new PredictionHistory(2, 0.001);
  const base = idleState();
  history.reset(base);
  for (let sequence = 1; sequence <= 4; sequence++) {
    history.predict(input(sequence), tuning, dt);
  }
  assert.equal(history.metrics.history_overflow_count, 2);
  const authoritative = { ...base, position: [0.25, 0, 0] };
  const state = history.reconcile(authoritative, 1, tuning, dt);
  assert.deepEqual(state.position, authoritative.position);
  assert.equal(history.metrics.hard_resync_count, 1);
  assert.equal(history.metrics.last_resync_reason, "prediction_history_overflow");
  assert.equal(history.metrics.discarded_input_count, 3);
  assert.equal(history.inputs.length, 0);
});

test("prediction history rejects duplicate input sequences and ignores stale ACKs", () => {
  const history = new PredictionHistory(8, 0.5);
  const base = { ...idleState(), last_processed_input: -1 };
  history.reset(base);
  history.predict(input(1), tuning, dt);
  history.predict(input(2), tuning, dt);
  assert.throws(() => history.predict(input(2), tuning, dt), /sequences must increase/);

  const ackState = history.inputs.at(-1).state;
  history.reconcile(structuredClone(ackState), 2, tuning, dt);
  const accepted = structuredClone(history.state);
  const staleState = { ...base, position: [100, 0, 100] };
  history.reconcile(staleState, 1, tuning, dt);
  assert.deepEqual(history.state, accepted);
  assert.equal(history.metrics.stale_ack_count, 1);
  assert.equal(history.metrics.reconciliation_count, 1);
  assert.throws(() => history.reconcile(ackState, 2.5, tuning, dt), /valid ACK/);
  assert.throws(
    () => history.reconcile(ackState, 3, tuning, dt),
    /cannot exceed the latest predicted input sequence/,
  );
  assert.throws(() => history.reconcile({ ...base, position: [Number.NaN, 0, 0] }, 2, tuning, dt), /authoritative transform/);
});

test("visual transform smoothing eases position and yaw while large corrections snap", () => {
  const smoothing = new VisualTransformSmoothing({ durationSeconds: 0.12, snapDistance: 3 });
  smoothing.correct(
    { position: [10, 0, 0], character_yaw: -179 },
    { position: [9, 0, 0], character_yaw: 179 },
  );
  const first = smoothing.render({ position: [9, 0, 0], character_yaw: 179 }, 0);
  const settled = smoothing.render({ position: [9, 0, 0], character_yaw: 179 }, 0.12);
  assert.deepEqual(first, { position: [10, 0, 0], character_yaw: -179 });
  assert.ok(settled.position[0] > 9 && settled.position[0] < 10);
  assert.ok(Math.abs(settled.character_yaw) > 179);
  smoothing.correct(
    { position: [20, 0, 0], character_yaw: 0 },
    { position: [9, 0, 0], character_yaw: 0 },
  );
  assert.deepEqual(smoothing.render({ position: [9, 0, 0], character_yaw: 0 }, 0), {
    position: [9, 0, 0],
    character_yaw: 0,
  });
  assert.equal(smoothing.lastCorrection, "snap");
});

test("visual transform smoothing supports linear and explicit snap modes", () => {
  const linear = new VisualTransformSmoothing({ durationSeconds: 0.2, snapDistance: 3, mode: "linear" });
  linear.correct({ position: [1, 0, 0], character_yaw: 90 }, { position: [0, 0, 0], character_yaw: 0 });
  assert.equal(linear.render({ position: [0, 0, 0], character_yaw: 0 }, 0.1).position[0], 0.5);
  const snap = new VisualTransformSmoothing({ mode: "snap" });
  snap.correct({ position: [1, 0, 0], character_yaw: 90 }, { position: [0, 0, 0], character_yaw: 0 });
  assert.deepEqual(snap.render({ position: [0, 0, 0], character_yaw: 0 }, 0), {
    position: [0, 0, 0],
    character_yaw: 0,
  });
});

test("remote snapshots interpolate with bounded tangents and cap extrapolation", () => {
  const buffer = new RemoteSnapshotBuffer(32, 10, { maxVisualVelocity: 16 });
  buffer.push({ tick: 0, position: [0, 0, 0], velocity: [60, 0, 0], character_yaw: 0 });
  buffer.push({ tick: 2, position: [2, 0, 0], velocity: [60, 0, 0], character_yaw: 10 });
  assert.deepEqual(buffer.sample(1, 60).position, [1, 0, 0]);
  assert.deepEqual(buffer.sample(100, 60, 0.1).position, [3.6, 0, 0]);
  buffer.push({ tick: 3, position: [30, 0, 0], velocity: [0, 0, 0], character_yaw: 0 });
  assert.equal(buffer.samples.length, 1);
  assert.equal(buffer.lastPushTeleported, true);
});

test("remote Hermite interpolation prevents stopping and pivot overshoot", () => {
  const buffer = new RemoteSnapshotBuffer(32, 10, { maxVisualVelocity: 20 });
  buffer.push({ tick: 0, position: [0, 0, 0], velocity: [0, 0, 20], character_yaw: 0, yaw_rate: 720 });
  buffer.push({ tick: 60, position: [1, 0, 0], velocity: [0, 0, -20], character_yaw: 90, yaw_rate: 720 });
  const sample = buffer.sample(30, 60);
  assert.ok(sample.position[0] >= 0 && sample.position[0] <= 1);
  assert.equal(sample.position[2], 5);
  assert.ok(Math.hypot(...sample.position) <= 20 * 0.5);
  assert.ok(sample.character_yaw >= 0 && sample.character_yaw <= 90);
  const stopped = new RemoteSnapshotBuffer();
  stopped.push({ tick: 0, position: [3, 0, 0], velocity: [10, 0, 0], character_yaw: 0 });
  stopped.push({ tick: 2, position: [3, 0, 0], velocity: [0, 0, 0], character_yaw: 0 });
  assert.deepEqual(stopped.sample(1, 60).position, [3, 0, 0]);
});

test("remote interpolation preserves lateral tangents and yaw rate across the angle seam", () => {
  const buffer = new RemoteSnapshotBuffer(8, 10, {
    maxVisualVelocity: 16,
    maxVisualYawRate: 180,
  });
  buffer.push({
    tick: 10,
    position: [0, 0, 0],
    velocity: [0, 0, 2],
    character_yaw: 179,
    yaw_rate: 60,
  });
  buffer.push({
    tick: 20,
    position: [1, 0, 0],
    velocity: [0, 0, -2],
    character_yaw: -179,
    yaw_rate: 60,
  });
  const middle = buffer.sample(15, 60);
  assert.ok(middle.position[2] > 0.07);
  assert.ok(middle.position[2] < 0.1);
  assert.ok(Math.abs(angleDelta(middle.character_yaw, 180)) < 1e-8);
  const before = buffer.sample(14, 60);
  const after = buffer.sample(16, 60);
  assert.ok(Math.abs(angleDelta(after.character_yaw, before.character_yaw)) < 1);
  assert.throws(() => buffer.push({ tick: 21, position: [0, 0], velocity: [0, 0, 0] }), /finite tick/);
});

test("server clock estimates current server tick from arrival and RTT", () => {
  const clock = new ServerClockEstimator(60);
  clock.observe(600, 10000, 100);
  assert.equal(clock.estimateTick(10000), 603);
  clock.observe(606, 10100, 100);
  assert.ok(Math.abs(clock.estimateTick(10100) - 609) < 0.01);
  assert.equal(clock.observationCount, 2);
  assert.equal(clock.observe(606, 100100, 100), false);
});

test("fixed-step input scheduler limits catch-up bursts and tolerates clock resets", () => {
  const scheduler = new FixedStepScheduler(60, 3);
  scheduler.reset(0);
  assert.deepEqual(scheduler.advance(8), []);
  const first = scheduler.advance(18);
  assert.equal(first.length, 1);
  assert.ok(Math.abs(first[0] - 1000 / 60) < 1e-6);

  const catchup = scheduler.advance(218);
  assert.equal(catchup.length, 3);
  for (let index = 1; index < catchup.length; index++) {
    assert.ok(Math.abs(catchup[index] - catchup[index - 1] - 1000 / 60) < 1e-6);
  }
  assert.deepEqual(scheduler.advance(100), []);
  assert.deepEqual(scheduler.advance(108), []);
  assert.equal(scheduler.advance(118).length, 1);
  assert.throws(() => scheduler.advance(Number.NaN), /time must be finite/);
});

test("adaptive interpolation delay follows snapshot cadence, jitter, and RTT variance", () => {
  const delay = new AdaptiveInterpolationDelay({ minMs: 30, maxMs: 220, initialMs: 50 });
  delay.observe(0, 0, 60, 40);
  delay.observe(3, 50, 60, 40);
  const steady = delay.delayMs;
  delay.observe(6, 100, 60, 40);
  assert.ok(delay.delayMs >= steady);
  delay.observe(9, 190, 60, 120);
  assert.ok(delay.delayMs > steady);
  for (let index = 12; index < 90; index += 3) delay.observe(index, index * 1000 / 60, 60, 40);
  assert.ok(delay.delayMs <= 220);
  assert.ok(delay.delayMs >= 30);
  assert.ok(delay.arrivalJitterMs > 0);
  assert.ok(delay.rttVarianceMs > 0);
});

test("collision prediction stops at cover and preserves tangential wall motion", () => {
  const collisionWorld = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [{ name: "cover", center: [0, 0.9, 0], half_extents: [1.1, 0.9, 1.1] }],
    ramps: [],
  });
  let state = {
    ...idleState(),
    position: [0, 0.9, -3],
    ground_contact_point: [0, 0, -3],
    simulation_tick: 1,
  };
  const sprintInput = (sequence) => ({ ...input(sequence), requested_gait: "sprint" });
  for (let sequence = 1; sequence <= 120; sequence++) {
    state = predictMovementStep(
      state,
      sprintInput(sequence),
      collisionTuning,
      dt,
      collisionWorld,
    );
  }
  assert.ok(state.position[2] <= -1.54);
  assert.equal(state.grounded, true);

  const history = new PredictionHistory(256, 0.5);
  const baseState = {
    ...idleState(),
    position: [0, 0.9, -3],
    ground_contact_point: [0, 0, -3],
    simulation_tick: 1,
  };
  history.reset(baseState);
  for (let sequence = 1; sequence <= 120; sequence++) {
    history.predict(sprintInput(sequence), collisionTuning, dt, collisionWorld);
  }
  const acknowledged = predictMovementStep(
    baseState,
    sprintInput(1),
    collisionTuning,
    dt,
    collisionWorld,
  );
  assert.ok(
    history.state.position[2] > acknowledged.position[2],
    JSON.stringify({ predicted: history.state.position, acknowledged: acknowledged.position }),
  );
  const replayed = history.reconcile(acknowledged, 1, collisionTuning, dt, collisionWorld);
  assertVector(replayed.position, state.position, "collision replay position");
  assertVector(replayed.velocity, state.velocity, "collision replay velocity");
  assert.equal(history.metrics.large_correction_count, 0, JSON.stringify(history.metrics));
  assert.equal(history.metrics.position_error, 0);
  assert.equal(history.metrics.correction_position_error, 0);
  assert.equal(state.actual_gait, "idle");
  assert.equal(createMotionFrame({ ...state, requested_gait: "sprint" }, 120).actualGait, "idle");

  const alongWall = collisionWorld.moveCharacter(
    { ...state, position: [1.56, 0.9, -3], blocked_move_ticks: 0 },
    [1.56, 0.9, -2.9],
    [0, 0, 6],
    collisionTuning,
    dt,
  );
  assert.ok(alongWall.position[2] > -3);
});

test("ground grace remains grounded but cannot predict a jump without confirmed contact", () => {
  const gapWorld = new CollisionWorld({ version: 1, planes: [], boxes: [], ramps: [] });
  const graceState = {
    ...idleState(),
    position: [0, 0.9, 0],
    simulation_tick: 3,
    last_grounded_tick: 2,
    grounded: true,
    ground_contact_confirmed: false,
    floor_distance: 0,
  };
  const jumped = predictMovementStep(
    graceState,
    { ...input(1, 0), jump: true },
    collisionTuning,
    dt,
    gapWorld,
  );
  assert.equal(jumped.grounded, true);
  assert.equal(jumped.ground_contact_confirmed, false);
  assert.equal(jumped.ground_grace_active, true);
  assert.notEqual(jumped.locomotion_phase, "jump_start");
  assert.ok(jumped.velocity[1] < 1);

  const history = new PredictionHistory(256, 0.001);
  history.reset(graceState);
  const predicted = history.predict(
    { ...input(1, 0), jump: true }, collisionTuning, dt, gapWorld,
  );
  history.reconcile(structuredClone(predicted), 1, collisionTuning, dt, gapWorld);
  assert.equal(history.metrics.large_correction_count, 0);
});

test("ramp side and high-end sweeps block penetration while preserving wall slides", () => {
  const ramp = {
    name: "walkable-ramp",
    center: [0, 0.95, 0],
    half_extents: [2, 0.15, 2],
    pitch_degrees: -20,
  };
  const floor = { name: "floor", normal: [0, 1, 0], constant: 0 };
  const rampWorld = new CollisionWorld({ version: 1, planes: [floor], boxes: [], ramps: [ramp] });
  const halfHeight = collisionTuning.character_radius +
    collisionTuning.character_cylinder_height / 2;
  const topAt = (z) => {
    const pitch = ramp.pitch_degrees * Math.PI / 180;
    const topY = ramp.center[1] + Math.cos(pitch) * ramp.half_extents[1];
    const topZ = ramp.center[2] + Math.sin(pitch) * ramp.half_extents[1];
    return topY - Math.tan(pitch) * (z - topZ);
  };

  const sideY = topAt(0) + halfHeight;
  const side = rampWorld.moveCharacter(
    { ...idleState(), position: [1.5, sideY, 0], ground_contact_point: [1.5, topAt(0), 0] },
    [3, sideY, 0],
    [90, 0, 0],
    collisionTuning,
    dt,
  );
  assert.ok(side.position[0] <= 1.56, JSON.stringify(side));
  assert.equal(side.grounded, true);

  const highEndStartZ = 1;
  const highEndY = topAt(highEndStartZ) + halfHeight;
  const highEnd = rampWorld.moveCharacter(
    {
      ...idleState(),
      position: [0, highEndY, highEndStartZ],
      ground_contact_point: [0, topAt(highEndStartZ), highEndStartZ],
    },
    [0, highEndY, 3],
    [0, 0, 120],
    collisionTuning,
    dt,
  );
  assert.ok(highEnd.position[2] < 1.5, JSON.stringify(highEnd));

  const wallWorld = new CollisionWorld({
    version: 1,
    planes: [floor],
    boxes: [{ name: "wall", center: [1.4, 1.5, 3], half_extents: [1.2, 1.5, 0.3] }],
    ramps: [ramp],
  });
  const combined = wallWorld.moveCharacter(
    { ...idleState(), position: [1.5, sideY, 0], ground_contact_point: [1.5, topAt(0), 0] },
    [3, sideY, 3],
    [90, 0, 180],
    collisionTuning,
    dt,
  );
  assert.ok(combined.position[0] <= 1.56, JSON.stringify(combined));
  assert.ok(combined.position[2] <= 2.26, JSON.stringify(combined));
});

test("foot probes find bounded floor, step, and ramp supports from collision geometry", () => {
  const world = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [{ name: "step", center: [0, 0.15, 0.2], half_extents: [1, 0.15, 0.5] }],
    ramps: [{
      name: "ramp",
      center: [4, 0.95, 0],
      half_extents: [2, 0.15, 2],
      pitch_degrees: -20,
    }],
  });
  const floor = world.probeFoot([4, 0.1, 4], collisionTuning);
  assert.equal(floor.grounded, true);
  assert.equal(floor.entity, "floor");
  assert.equal(floor.distance, 0.1);

  const step = world.probeFoot([0, 0.3, 0.2], collisionTuning);
  assert.equal(step.grounded, true);
  assert.equal(step.entity, "step");
  assert.deepEqual(step.normal, [0, 1, 0]);
  assert.equal(world.probeFoot([0, 0.7, 0.2], collisionTuning).grounded, false);

  const ramp = world.probeFoot([4, 1.2, 0], collisionTuning, { maxDistance: 1 });
  assert.equal(ramp.entity, "ramp");
  assert.ok(ramp.normal[1] > 0.9);
  assert.throws(() => world.probeFoot([0, Number.NaN, 0], collisionTuning), /finite position/);
});

test("continuous ramp movement does not accumulate ACK-timeline corrections", () => {
  const ramp = {
    name: "walkable-ramp",
    center: [0, 0.95, 0],
    half_extents: [2, 0.15, 2],
    pitch_degrees: -20,
  };
  const rampWorld = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [],
    ramps: [ramp],
  });
  const halfHeight = collisionTuning.character_radius +
    collisionTuning.character_cylinder_height / 2;
  const pitch = ramp.pitch_degrees * Math.PI / 180;
  const surfaceY = ramp.center[1] + Math.cos(pitch) * ramp.half_extents[1] -
    Math.tan(pitch) * (-1 - Math.sin(pitch) * ramp.half_extents[1]);
  const base = {
    ...idleState(),
    position: [0, surfaceY + halfHeight, -1],
    ground_contact_point: [0, surfaceY, -1],
    simulation_tick: 1,
  };
  const history = new PredictionHistory(256, 0.001);
  history.reset(base);
  for (let sequence = 1; sequence <= 180; sequence++) {
    history.predict(input(sequence), collisionTuning, dt, rampWorld);
    if (sequence % 3 === 0) {
      const acknowledged = history.inputs.find(({ input: pending }) =>
        pending.sequence === sequence)?.state;
      assert.ok(acknowledged, `missing predicted ACK state ${sequence}`);
      history.reconcile(structuredClone(acknowledged), sequence, collisionTuning, dt, rampWorld);
    }
  }
  assert.equal(history.metrics.reconciliation_count, 60);
  assert.equal(history.metrics.large_correction_count, 0, JSON.stringify(history.metrics));
  assert.equal(history.metrics.position_error, 0);
  assert.equal(history.metrics.correction_position_error, 0);
});

test("collision prediction steps up a low box and lands after falling", () => {
  const stepWorld = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [{ name: "step", center: [0, 0.15, 0.2], half_extents: [1, 0.15, 0.5] }],
    ramps: [],
  });
  let state = {
    ...idleState(),
    position: [0, 0.9, -1],
    ground_contact_point: [0, 0, -1],
    simulation_tick: 1,
  };
  let stepped = false;
  for (let sequence = 1; sequence <= 60; sequence++) {
    state = predictMovementStep(
      state,
      input(sequence),
      collisionTuning,
      dt,
      stepWorld,
    );
    if (state.step_up) {
      stepped = true;
      break;
    }
  }
  assert.equal(stepped, true);
  assert.equal(state.ground_entity, "step");
  assert.ok(state.position[1] >= 1.2);
  const stepHeight = state.position[1];
  let returnedToFloor = false;
  for (let sequence = 61; sequence <= 120; sequence++) {
    state = predictMovementStep(
      state,
      input(sequence, -1),
      collisionTuning,
      dt,
      stepWorld,
    );
    if (state.ground_entity === "floor" && state.position[1] < stepHeight - 0.05) {
      returnedToFloor = true;
      break;
    }
  }
  assert.equal(returnedToFloor, true);

  const floorWorld = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [],
    ramps: [],
  });
  state = {
    ...idleState(),
    position: [0, 3, 0],
    velocity: [0, -2, 0],
    vertical_speed: -2,
    grounded: false,
    ground_contact_confirmed: false,
    movement_mode: "airborne",
    locomotion_phase: "falling",
    simulation_tick: 1,
  };
  let landed = false;
  for (let sequence = 1; sequence <= 120; sequence++) {
    state = predictMovementStep(
      state,
      input(sequence, 0),
      collisionTuning,
      dt,
      floorWorld,
    );
    if (state.grounded) {
      landed = true;
      break;
    }
  }
  assert.equal(landed, true);
  assert.ok(Math.abs(state.position[1] - 0.9) < 1e-6);
  assert.ok(["soft_land", "normal_land", "heavy_land"].includes(state.locomotion_phase));
});

test("bounded step solving handles supported risers without adding a forward lunge", () => {
  const runStep = (height, extraBoxes = []) => {
    const stepWorld = new CollisionWorld({
      version: 1,
      planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
      boxes: [
        { name: "step", center: [0, height / 2, 0.2], half_extents: [1, height / 2, 0.5] },
        ...extraBoxes,
      ],
      ramps: [],
    });
    let state = {
      ...idleState(),
      position: [0, 0.9, -1],
      ground_contact_point: [0, 0, -1],
      simulation_tick: 1,
    };
    let maxTickDisplacement = 0;
    for (let sequence = 1; sequence <= 90; sequence++) {
      const previous = state.position;
      state = predictMovementStep(state, input(sequence), collisionTuning, dt, stepWorld);
      maxTickDisplacement = Math.max(
        maxTickDisplacement,
        Math.hypot(
          state.position[0] - previous[0],
          state.position[2] - previous[2],
        ),
      );
      if (state.step_up) break;
    }
    return { state, maxTickDisplacement };
  };

  for (const height of [0.1, 0.2, collisionTuning.character_step_height]) {
    const result = runStep(height);
    assert.equal(result.state.step_up, true, `step height ${height}`);
    assert.equal(result.state.ground_entity, "step");
    assert.ok(Math.abs(result.state.position[1] - (0.9 + height)) <= 0.002);
    assert.ok(
      result.maxTickDisplacement <= collisionTuning.run_speed * dt + 0.002,
      `step height ${height} moved ${result.maxTickDisplacement}m in one tick`,
    );
  }

  const tooHigh = runStep(collisionTuning.character_step_height + 0.01);
  assert.equal(tooHigh.state.step_up, false);
  assert.ok(tooHigh.state.position[2] < -0.7);

  const overhead = runStep(0.2, [
    { name: "overhead", center: [0, 2.02, 0.2], half_extents: [1, 0.2, 0.5] },
  ]);
  assert.equal(overhead.state.step_up, false);
  assert.ok(overhead.state.position[2] < -0.7);
});

test("step traversal stays bounded beside walls, on diagonals, and across repeated stairs", () => {
  const makeWorld = (boxes) => new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes,
    ramps: [],
  });
  const stair = { name: "step", center: [0, 0.1, 0.2], half_extents: [1, 0.1, 0.5] };
  const besideWall = makeWorld([
    stair,
    { name: "wall", center: [1.3, 0.9, 0.2], half_extents: [0.2, 0.9, 0.5] },
  ]);
  let state = {
    ...idleState(),
    position: [0, 0.9, -1],
    ground_contact_point: [0, 0, -1],
    simulation_tick: 1,
  };
  let steppedBesideWall = false;
  for (let sequence = 1; sequence <= 120; sequence++) {
    const previous = state.position;
    state = predictMovementStep(
      state,
      { ...input(sequence), move_x: 0.7 },
      collisionTuning,
      dt,
      besideWall,
    );
    assert.ok(
      Math.hypot(state.position[0] - previous[0], state.position[2] - previous[2])
        <= collisionTuning.run_speed * dt + 0.002,
    );
    if (state.step_up) {
      steppedBesideWall = true;
      break;
    }
  }
  assert.equal(steppedBesideWall, true);
  assert.ok(state.position[0] < 0.71, JSON.stringify(state));

  const diagonalWorld = makeWorld([{
    name: "diagonal-step",
    center: [0, 0.15, 0.2],
    half_extents: [0.75, 0.15, 0.5],
  }]);
  state = {
    ...idleState(),
    position: [0, 0.9, -1],
    ground_contact_point: [0, 0, -1],
    simulation_tick: 1,
  };
  let steppedDiagonally = false;
  for (let sequence = 1; sequence <= 150; sequence++) {
    const previous = state.position;
    state = predictMovementStep(
      state,
      { ...input(sequence), move_x: 0.25, move_z: 1 },
      collisionTuning,
      dt,
      diagonalWorld,
    );
    assert.ok(
      Math.hypot(state.position[0] - previous[0], state.position[2] - previous[2])
        <= collisionTuning.run_speed * dt + 0.002,
    );
    if (state.step_up) {
      steppedDiagonally = true;
      break;
    }
  }
  assert.equal(steppedDiagonally, true, JSON.stringify(state));

  const stairs = [0.1, 0.2, 0.3].map((height, index) => ({
    name: `stair-${index + 1}`,
    center: [0, height / 2, 0.2 + index * 0.5],
    half_extents: [1, height / 2, 0.5],
  }));
  const stairWorld = makeWorld(stairs);
  const base = {
    ...idleState(),
    position: [0, 0.9, -1],
    ground_contact_point: [0, 0, -1],
    simulation_tick: 1,
  };
  const history = new PredictionHistory(256, 0.001);
  history.reset(base);
  let reachedThirdStair = false;
  for (let sequence = 1; sequence <= 180; sequence++) {
    const previous = history.state.position;
    const predicted = history.predict(input(sequence), collisionTuning, dt, stairWorld);
    assert.ok(
      Math.hypot(predicted.position[0] - previous[0], predicted.position[2] - previous[2])
        <= collisionTuning.run_speed * dt + 0.002,
    );
    if (predicted.ground_entity === "stair-3") reachedThirdStair = true;
  }
  const predictedEnd = structuredClone(history.state);
  const ack = history.inputs.find(({ input: pending }) => pending.sequence === 1)?.state;
  assert.ok(ack);
  const replayed = history.reconcile(structuredClone(ack), 1, collisionTuning, dt, stairWorld);
  assert.equal(reachedThirdStair, true);
  assert.deepEqual(replayed.position, predictedEnd.position);
  assert.equal(history.metrics.large_correction_count, 0);
});

test("MotionFrame carries animation channels and bounded motion history", () => {
  const state = {
    ...idleState(),
    velocity: [1, 0, 2],
    horizontal_speed: Math.sqrt(5),
    character_yaw: 15,
    locomotion_phase: "turn_in_place",
    phase_start_tick: 90,
    phase_duration_ticks: 20,
    turn_angle: 90,
    remaining_turn_angle: -49.5,
    turn_direction: "left",
    turn_progress: 0.45,
    action_channels: {
      additive_reaction: channelSnapshot("hit_reaction", true, 1),
    },
    gait_phase: 0.98,
  };
  const frame = createMotionFrame(state, 100, dt);
  assert.equal(frame.tick, 100);
  assert.equal(frame.phaseProgress, 0.5);
  assert.equal(frame.turnDirection, "left");
  assert.equal(frame.turnProgress, 0.45);
  assert.equal(frame.remainingTurnAngle, -49.5);
  assert.equal(frame.gaitPhase, 0.98);
  assert.deepEqual(frame.actionLayers.map(({ channel, state }) => [channel, state]), [
    ["additive_reaction", "hit_reaction"],
  ]);
  assert.equal(frame.actionChannels.additive_reaction.active, true);
  assert.equal(frame.footIK.leftFootGroundDistance, null);
  assert.ok(Math.abs(frame.movementDirection - (Math.atan2(1, 2) * 180 / Math.PI - 15)) < 1e-9);
  const motionHistory = new MotionHistory(2);
  motionHistory.push(frame);
  motionHistory.push(frame);
  motionHistory.push(frame);
  assert.equal(motionHistory.latest().length, 2);
});

test("action channel cursor deduplicates events and rejects stale or reused revisions", () => {
  const cursor = new ActionChannelSnapshotCursor();
  const baseline = {
    locomotion: channelSnapshot("grounded:run:loop", true, 1),
    upper_body_action: channelSnapshot("shoot", true, 1),
    additive_reaction: channelSnapshot("hit_reaction", true, 1),
    full_body_override: channelSnapshot(),
    life_override: channelSnapshot(),
  };
  const first = cursor.apply(baseline);
  assert.deepEqual(first.startedEvents.map(({ channel }) => channel), [
    "upper_body_action", "additive_reaction",
  ]);
  assert.equal(cursor.apply(baseline).startedEvents.length, 0);

  const newer = {
    ...baseline,
    upper_body_action: channelSnapshot("none", false, 2),
    additive_reaction: channelSnapshot("hit_reaction", true, 2),
  };
  const advanced = cursor.apply(newer);
  assert.deepEqual(advanced.startedEvents.map(({ channel }) => channel), ["additive_reaction"]);
  const reordered = cursor.apply(baseline);
  assert.equal(reordered.startedEvents.length, 0);
  assert.equal(reordered.channels.upper_body_action.active, false);
  assert.equal(reordered.channels.additive_reaction.sequence, 2);

  assert.throws(() => cursor.apply({
    ...newer,
    additive_reaction: { ...newer.additive_reaction, active: false },
  }), /reused sequence/);
});

test("animation graph returns normalized blend, additive aim, and warp semantics", () => {
  const frame = createMotionFrame({
    ...idleState(),
    horizontal_speed: 4.5,
    velocity: [0, 0, 4.5],
    aim_yaw: 220,
    aim_pitch: -95,
    action_channels: {
      additive_reaction: channelSnapshot("hit_reaction", true, 1),
    },
    hit_strength: 0.6,
  }, 10);
  const weights = evaluateBlendSpace(frame, tuning.sprint_speed);
  assert.ok(Math.abs(Object.values(weights).reduce((sum, value) => sum + value, 0) - 1) < 1e-9);
  const aim = evaluateAimOffset(220, -95);
  assert.equal(aim.yaw, 90);
  assert.equal(aim.pitch, -60);
  assert.equal(Object.values(aim.weights).reduce((sum, weight) => sum + weight, 0), 1);
  assert.equal(Math.abs(orientationWarpAngle(180, 0)), 90);
  const graph = evaluateAnimationGraph(frame, tuning, new BlendWeightSmoothing(), dt);
  assert.equal(graph.hitReaction, 0.6);
  assert.equal(graph.phase, "idle");
});

test("blend space selects local samples with a circular direction axis", () => {
  const positiveSeam = evaluateBlendSpace({ movementDirection: 179, horizontalSpeed: 3 }, 6.5);
  const negativeSeam = evaluateBlendSpace({ movementDirection: -179, horizontalSpeed: 3 }, 6.5);
  const total = Object.values(positiveSeam).reduce((sum, weight) => sum + weight, 0);
  assert.ok(Math.abs(total - 1) < 1e-12);
  assert.ok(Object.values(positiveSeam).every((weight) => weight >= 0));
  for (const name of ["idle", "walk_forward", "walk_backward", "run_forward", "run_backward", "sprint"]) {
    assert.ok(Math.abs(positiveSeam[name] - negativeSeam[name]) < 0.04, name);
  }
  assert.ok(Math.abs(positiveSeam.walk_left - negativeSeam.walk_right) < 0.04);
  assert.ok(Math.abs(positiveSeam.walk_right - negativeSeam.walk_left) < 0.04);
  assert.ok(Math.abs(positiveSeam.run_left - negativeSeam.run_right) < 0.04);
  assert.ok(Math.abs(positiveSeam.run_right - negativeSeam.run_left) < 0.04);
  assert.equal(Object.values(positiveSeam).filter((weight) => weight > 0).length, 3);
});

test("blend space has exact walk and run samples for eight movement directions", () => {
  const directions = [
    ["forward", 0], ["forward_right", 45], ["right", 90], ["backward_right", 135],
    ["backward", 180], ["backward_left", -135], ["left", -90], ["forward_left", -45],
  ];
  for (const [gait, speed] of [["walk", 2], ["run", 4.5]]) {
    for (const [direction, degrees] of directions) {
      const weights = evaluateBlendSpace({ movementDirection: degrees, horizontalSpeed: speed }, 6.5);
      const selected = `${gait}_${direction}`;
      assert.equal(weights[selected], 1, `${selected} at ${degrees} degrees`);
      assert.equal(Object.values(weights).filter((weight) => weight > 0).length, 1);
    }
  }
});

test("blend weight smoothing clamps and normalizes each result", () => {
  const smoother = new BlendWeightSmoothing(0.08);
  const target = { idle: 1, run: 0, sprint: 0 };
  assert.deepEqual(smoother.update(target, dt), target);
  for (let index = 0; index < 30; index++) {
    const weights = smoother.update({ idle: 0, run: 0, sprint: 1 }, dt);
    assert.ok(Object.values(weights).every((weight) => weight >= 0));
    assert.ok(Math.abs(Object.values(weights).reduce((sum, weight) => sum + weight, 0) - 1) < 1e-12);
  }
});

test("synthetic skeleton clip sampling and quaternion pose blending produce world transforms", () => {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "spine", parentIndex: 0, bindLocal: createTransform([0, 1, 0]) },
  ]);
  const idlePose = createPose(skeleton);
  const clip = new AnimationClip("spine-turn", 1, [
    new AnimationTrack(1, [
      new Keyframe(0, createTransform([0, 1, 0])),
      new Keyframe(1, createTransform([0, 1, 0], [0, Math.sin(Math.PI / 4), 0, Math.cos(Math.PI / 4)])),
    ]),
  ]);
  const halfPose = sampleAnimationClip(clip, skeleton, 0.5);
  const sampler = new ClipSampler(clip, skeleton);
  assert.deepEqual(sampler.sample(0.5).localTransforms, halfPose.localTransforms);
  const blended = blendPoses(idlePose, halfPose, 0.5);
  assert.equal(blended.skeleton, skeleton);
  assert.ok(Math.abs(Math.hypot(...blended.localTransforms[1].rotation) - 1) < 1e-12);
  assert.ok(Math.abs(blended.localTransforms[1].rotation[1]) > 0);
  const weighted = blendWeightedPoses([
    { pose: idlePose, weight: 1 },
    { pose: halfPose, weight: 3 },
  ]);
  const worlds = worldTransforms(weighted);
  assert.equal(worlds.length, 2);
  assert.ok(Math.abs(worlds[1].translation[1] - 1) < 1e-9);
  assert.ok(Math.abs(Math.hypot(...quaternionSlerp([0, 0, 0, 1], [0, 1, 0, 0], 0.5)) - 1) < 1e-12);
});

test("pose inertializer preserves transform continuity and decays real pose offsets", () => {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "pelvis", parentIndex: 0 },
  ]);
  const outgoing = createPose(skeleton, [
    createTransform([0.2, 0, 0]),
    createTransform([0, 1.2, 0], [0, Math.sin(0.2), 0, Math.cos(0.2)]),
  ]);
  const incoming = createPose(skeleton, [
    createTransform(),
    createTransform([0, 1, 0]),
  ]);
  const inertializer = new PoseInertializer(0.12);
  inertializer.begin(outgoing, incoming);
  const atTransition = inertializer.sample(incoming, 0);
  const settled = inertializer.sample(incoming, 1.2);
  assert.deepEqual(atTransition.localTransforms, outgoing.localTransforms);
  assert.ok(Math.abs(settled.localTransforms[1].translation[1] - 1) < 1e-5);
  assert.ok(Math.abs(settled.localTransforms[1].rotation[1]) < 1e-5);
});

test("2D aim offset blends additive pose samples through an upper body mask", () => {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "spine", parentIndex: 0 },
    { name: "left_leg", parentIndex: 0 },
  ]);
  const identity = createPose(skeleton);
  const rightAim = createPose(skeleton, [
    createTransform(),
    createTransform([0, 0, 0], [0, Math.sin(Math.PI / 8), 0, Math.cos(Math.PI / 8)]),
    createTransform(),
  ]);
  const aimSamples = Object.fromEntries([
    "down_left", "down", "down_right", "left", "center", "right", "up_left", "up", "up_right",
  ].map((name) => [name, name === "right" ? rightAim : identity]));
  const upperBody = BoneMask.fromNames(skeleton, ["spine"]);
  const aimed = evaluateAimOffsetPose(identity, aimSamples, 90, 0, 1, upperBody);
  assert.ok(Math.abs(aimed.localTransforms[1].rotation[1]) > 0);
  assert.deepEqual(aimed.localTransforms[2], identity.localTransforms[2]);
});

test("animation layer stack composes locomotion, upper body action, hit additive, and life override", () => {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "spine", parentIndex: 0 },
    { name: "left_leg", parentIndex: 0 },
  ]);
  const baseRun = createPose(skeleton, [
    createTransform(),
    createTransform([0, 1, 0]),
    createTransform([0, -1, 0]),
  ]);
  const shoot = createPose(skeleton, [
    createTransform(),
    createTransform([0, 1.2, 0]),
    createTransform([0, -3, 0]),
  ]);
  const hit = createPose(skeleton, [
    createTransform(),
    createTransform([0.2, 0, 0]),
    createTransform(),
  ]);
  const dead = createPose(skeleton, [
    createTransform([0, -0.8, 0]),
    createTransform([0, 0.4, 0]),
    createTransform([0, -1.2, 0]),
  ]);
  const upperBody = BoneMask.fromNames(skeleton, ["spine"]);
  const layers = new CharacterPoseLayerStack();
  layers.set("shoot", {
    channel: AnimationLayerChannel.UPPER_BODY_ACTION,
    mode: "override",
    pose: shoot,
    weight: 1,
    boneMask: upperBody,
  });
  layers.set("hit-reaction", {
    channel: AnimationLayerChannel.ADDITIVE_REACTION,
    mode: "additive",
    pose: hit,
    weight: 0.5,
  });
  const alivePose = layers.compose(baseRun);
  assert.equal(alivePose.localTransforms[1].translation[1], 1.2);
  assert.equal(alivePose.localTransforms[2].translation[1], -1);
  assert.ok(alivePose.localTransforms[1].translation[0] > 0);
  layers.set("death", {
    channel: AnimationLayerChannel.LIFE_OVERRIDE,
    mode: "override",
    pose: dead,
    weight: 1,
  });
  assert.deepEqual(layers.compose(baseRun).localTransforms, dead.localTransforms);
  assert.deepEqual(layers.activeLayers().map(({ name }) => name), ["shoot", "hit-reaction", "death"]);
  assert.equal(layers.remove("death"), true);
});

test("pose animation graph evaluates motion weights, additive aim, and action pose layers", () => {
  const skeleton = createSkeleton([
    { name: "root", parentIndex: -1 },
    { name: "spine", parentIndex: 0 },
  ]);
  const neutral = createPose(skeleton);
  const raised = createPose(skeleton, [
    createTransform(),
    createTransform([0, 0.25, 0]),
  ]);
  const tuningForGraph = { ...tuning, sprint_speed: 6.5 };
  const frame = createMotionFrame({
    ...idleState(),
    velocity: [0, 0, 2.5],
    horizontal_speed: 2.5,
    aim_yaw: 30,
    aim_pitch: 0,
    action_channels: {
      upper_body_action: channelSnapshot("shoot", true, 1),
      additive_reaction: channelSnapshot("hit_reaction", true, 1),
    },
  }, 20);
  const poseLibrary = {
    locomotionPoses: Object.fromEntries([
      "idle",
      ...["forward", "forward_right", "right", "backward_right", "backward", "backward_left", "left", "forward_left"]
        .flatMap((direction) => [`walk_${direction}`, `run_${direction}`]),
      "sprint",
    ].map((name) => [name, name.startsWith("walk") ? raised : neutral])),
    aimOffsetPoses: Object.fromEntries([
      "down_left", "down", "down_right", "left", "center", "right", "up_left", "up", "up_right",
    ].map((name) => [name, name === "right" ? raised : neutral])),
    layers: [{
      name: "test-hit",
      channel: AnimationLayerChannel.ADDITIVE_REACTION,
      mode: "additive",
      pose: raised,
      weight: 0.5,
    }],
    actionPoses: {
      upper_body_action: {
        shoot: {
          channel: AnimationLayerChannel.UPPER_BODY_ACTION,
          mode: "override",
          pose: raised,
          weight: 1,
          boneMask: BoneMask.fromNames(skeleton, ["spine"]),
        },
      },
      additive_reaction: {
        hit_reaction: {
          channel: AnimationLayerChannel.ADDITIVE_REACTION,
          mode: "additive",
          pose: raised,
          weight: 0.25,
        },
      },
    },
  };
  const graph = evaluatePoseAnimationGraph(
    frame,
    tuningForGraph,
    new BlendWeightSmoothing(),
    poseLibrary,
    dt,
  );
  assert.ok(graph.pose instanceof Pose);
  assert.ok(graph.selectedLocomotionSamples.length <= 3);
  assert.deepEqual(graph.activePoseLayers.map(({ name }) => name), [
    "test-hit", "action:upper_body_action:shoot", "action:additive_reaction:hit_reaction",
  ]);
  assert.ok(graph.pose.localTransforms[1].translation[1] > 0);
});

test("foot IK follows locked world contacts on uneven ground and aligns the foot to the slope", () => {
  const { makePose } = createMotionRig();
  const pose = makePose();
  const lock = new FootLockState();
  const lockedLeft = lock.lock("left", [0, 0.1, 0.2]);
  assert.deepEqual(lock.lock("left", [9, 9, 9]), lockedLeft);
  const rightLock = lock.lock("right", [0.2, 0.3, 0]);
  const probes = {
    left: { grounded: true, position: [8, 8, 8], lockedPosition: lockedLeft, normal: [0, 1, 0.5] },
    right: { grounded: true, position: [8, 8, 8], lockedPosition: rightLock, normal: [0, 1, 0] },
  };
  const result = solveFootIK(pose, probes);
  const worlds = worldTransforms(result.pose);
  const left = worlds[pose.skeleton.indexByName.get("left_foot")];
  const right = worlds[pose.skeleton.indexByName.get("right_foot")];
  for (let axis = 0; axis < 3; axis++) {
    assert.ok(Math.abs(left.translation[axis] - lockedLeft[axis]) < 1e-8);
    assert.ok(Math.abs(right.translation[axis] - rightLock[axis]) < 1e-8);
  }
  const expectedNormal = [0, 1 / Math.sqrt(1.25), 0.5 / Math.sqrt(1.25)];
  const actualNormal = rotateVector(left.rotation, [0, 1, 0]);
  for (let axis = 0; axis < 3; axis++) assert.ok(Math.abs(actualNormal[axis] - expectedNormal[axis]) < 1e-8);
  assert.ok(Math.abs(result.pelvisOffset - 0.1) < 1e-8);
  assert.deepEqual(result.footNormals.left, expectedNormal);
  lock.release("left");
  assert.equal(lock.target("left"), null);
  assert.deepEqual(lock.target("right"), rightLock);
});

test("orientation warp distributes yaw through the synthetic skeleton pose", () => {
  const { makePose, skeleton } = createMotionRig();
  const result = warpPoseOrientation(makePose(), 90, 0, {
    root: 0.1,
    pelvis: 0.15,
    spine: 0.2,
    left_foot: 0.15,
    right_foot: 0.15,
    chest: 0.25,
  });
  const worlds = worldTransforms(result.pose);
  const yawOf = (rotation) => 2 * Math.atan2(rotation[1], rotation[3]) * 180 / Math.PI;
  assert.equal(result.warpAngle, 90);
  assert.ok(Math.abs(yawOf(worlds[skeleton.indexByName.get("root")].rotation) - 9) < 1e-8);
  assert.ok(Math.abs(yawOf(worlds[skeleton.indexByName.get("pelvis")].rotation) - 22.5) < 1e-8);
  assert.ok(Math.abs(yawOf(worlds[skeleton.indexByName.get("spine")].rotation) - 40.5) < 1e-8);
  assert.ok(Math.abs(yawOf(worlds[skeleton.indexByName.get("chest")].rotation) - 63) < 1e-8);
  assert.ok(Math.abs(yawOf(worlds[skeleton.indexByName.get("left_foot")].rotation) - 36) < 1e-8);
  assert.throws(() => warpPoseOrientation(makePose(), 90, 0, { root: 0.5 }), /sum to one/);
  assert.throws(() => warpPoseOrientation(makePose(), 0, 0, { root: 1, missing: 0 }), /missing/);
  assert.equal(orientationWarpAngle(179, -179), -2);
  assert.equal(orientationWarpAngle(-179, 179), 2);
});

test("visual root offset remains bounded and root motion maps clip-local deltas into simulation facing", () => {
  const { makePose } = createMotionRig();
  const displaced = makePose({ rootPosition: [3, 2, 0] });
  const offset = applyVisualRootOffset(displaced, [0, 0, 0], { maxOffset: 5 });
  const alignedRoot = worldTransforms(offset.pose)[0].translation;
  assert.deepEqual(alignedRoot, [0, 0, 0]);
  assert.equal(offset.clamped, false);
  const clamped = applyVisualRootOffset(displaced, [0, 0, 0], { maxOffset: 0.5 });
  assert.equal(clamped.clamped, true);
  assert.ok(Math.abs(Math.hypot(...clamped.offset) - 0.5) < 1e-8);

  const previous = makePose({ rootYaw: 90 });
  const current = makePose({ rootPosition: [0, 0, -1], rootYaw: 120 });
  const rootDelta = extractRootMotionDelta(previous, current);
  assert.ok(Math.abs(rootDelta.translation[0] - 1) < 1e-8);
  assert.ok(Math.abs(rootDelta.translation[2]) < 1e-8);
  assert.ok(Math.abs(rootDelta.yawDelta - 30) < 1e-8);
  const applied = applyRootMotionDelta({ position: [10, 2, 10], character_yaw: 90 }, rootDelta);
  assert.ok(Math.abs(applied.position[0] - 10) < 1e-8);
  assert.ok(Math.abs(applied.position[1] - 2) < 1e-8);
  assert.ok(Math.abs(applied.position[2] - 9) < 1e-8);
  assert.equal(applied.character_yaw, 120);
});

test("root motion warping distributes endpoint correction across a synthetic traversal clip", () => {
  const target = new MotionWarpTarget("vault-landing", [8, 0, 0], 0);
  let current = [0, 0, 0];
  const warpedSteps = [];
  for (const remainingX of [3, 2, 1, 0]) {
    const result = warpRootMotionDelta(
      { translation: [1, 0, 0], yawDelta: 0 },
      { position: current, character_yaw: 0 },
      [remainingX, 0, 0],
      target,
    );
    warpedSteps.push(result.translation[0]);
    current = current.map((value, axis) => value + result.translation[axis]);
  }
  assert.ok(warpedSteps.every((step) => step > 1));
  assert.ok(Math.abs(current[0] - target.position[0]) < 1e-8);
});

test("root motion warping transforms clip-local deltas before solving a world-space target", () => {
  const target = new MotionWarpTarget("sideways-vault-landing", [8, 0, 0], 90);
  let transform = { position: [0, 0, 0], character_yaw: 90 };
  for (const remainingZ of [3, 2, 1, 0]) {
    const delta = warpRootMotionDelta(
      { translation: [0, 0, 1], yawDelta: 0 },
      transform,
      [0, 0, remainingZ],
      target,
    );
    transform = applyRootMotionDelta(transform, delta);
  }
  assert.ok(Math.abs(transform.position[0] - target.position[0]) < 1e-8);
  assert.ok(Math.abs(transform.position[2] - target.position[2]) < 1e-8);
  assert.equal(transform.character_yaw, target.yaw);
});

test("root motion warp is restricted by a smooth action-time window", () => {
  const window = new MotionWarpWindow(0.1, 0.9, { blendInSeconds: 0.2, blendOutSeconds: 0.2 });
  assert.equal(window.weightAt(0.05), 0);
  assert.equal(window.weightAt(0.1), 0);
  assert.ok(window.weightAt(0.2) > 0 && window.weightAt(0.2) < 1);
  assert.equal(window.weightAt(0.5), 1);
  assert.equal(window.weightAt(0.9), 0);
  assert.equal(window.weightAt(1), 0);
  assert.throws(() => new MotionWarpWindow(1, 0), /bounds and blends/);

  const target = new MotionWarpTarget("windowed", [10, 0, 0], 90, window);
  const args = [
    { translation: [1, 0, 0], yawDelta: 0 },
    { position: [0, 0, 0], character_yaw: 0 },
    [4, 0, 0],
    target,
  ];
  assert.throws(() => warpRootMotionDelta(...args), /requires root delta/);
  const outside = warpRootMotionDelta(...args, 1, 0, 1.1);
  assert.deepEqual(outside.translation, [1, 0, 0]);
  assert.equal(outside.yawDelta, 0);
  assert.equal(outside.effectiveWeight, 0);
  const inside = warpRootMotionDelta(...args, 1, 0, 0.5);
  assert.ok(inside.translation[0] > outside.translation[0]);
  assert.equal(inside.effectiveWeight, 1);
});

test("trajectory rollout replays future intent through the collision solver", () => {
  const wallWorld = new CollisionWorld({
    version: 1,
    planes: [{ name: "floor", normal: [0, 1, 0], constant: 0 }],
    boxes: [{ name: "wall", center: [0, 1, 3], half_extents: [2, 1, 0.1] }],
    ramps: [],
  });
  const start = { ...idleState(), position: [0, 0.9, 0], ground_contact_point: [0, 0, 0] };
  const samples = rolloutTrajectory(start, input(1), collisionTuning, dt, wallWorld, [0.2, 0.5, 1]);
  assert.equal(samples.length, 3);
  assert.ok(samples[0].position[2] < samples[1].position[2]);
  assert.ok(samples[1].position[2] <= samples[2].position[2]);
  assert.ok(samples[2].position[2] <= 2.46);
  assert.equal(samples[2].movementMode, "grounded");

  const openWorld = new CollisionWorld({ version: 1, planes: [], boxes: [], ramps: [] });
  const reverseIntent = (elapsed) => input(0, elapsed < 0.45 ? 1 : -1);
  const reversed = rolloutTrajectory(start, reverseIntent, collisionTuning, dt, openWorld, [0.4, 1]);
  const continued = rolloutTrajectory(start, input(0), collisionTuning, dt, openWorld, [0.4, 1]);
  assert.ok(reversed[1].position[2] < continued[1].position[2]);
  assert.equal(reversed[0].time, 0.4);
});

test("pose history stores sampled bones and pose search selects trajectory-consistent synthetic motion", () => {
  const { makePose } = createMotionRig();
  const trajectoryFor = (z, speed, facing = 0) => [{
    time: 0.2,
    position: [0, 0, z],
    velocity: [0, 0, speed],
    facing,
  }];
  const makeCandidate = (
    clipName,
    pose,
    velocity,
    trajectory,
    timeSeconds = 0,
    contacts = { left: false, right: false },
  ) => {
    const history = new PoseHistory(2);
    return history.push(pose, {
      tick: 1,
      rootVelocity: velocity,
      trajectory,
      contacts,
      clipName,
      timeSeconds,
    });
  };
  const idlePose = makePose();
  const runPose = makePose({ leftFoot: [-0.2, -1, 0.2], rightFoot: [0.2, -1, -0.2] });
  const pivotPose = makePose({ rootYaw: 180, leftFoot: [-0.2, -1, -0.25], rightFoot: [0.2, -1, 0.25] });
  const samples = [
    makeCandidate("idle", idlePose, [0, 0, 0], trajectoryFor(0, 0)),
    makeCandidate("run-forward", runPose, [0, 0, 3], trajectoryFor(0.6, 3), 0, {
      left: true,
      right: false,
    }),
    makeCandidate("pivot-back", pivotPose, [0, 0, -2], trajectoryFor(-0.4, -2, 180), 0, {
      left: false,
      right: true,
    }),
  ];
  const database = new PoseDatabase(samples.map((sample) => ({
    id: sample.id,
    clipName: sample.clipName,
    timeSeconds: sample.timeSeconds,
    pose: sample.pose,
    features: sample.features,
    metadata: { durationSeconds: 1 },
  })));
  const inconsistentFeatures = {
    ...samples[0].features,
    vector: samples[0].features.vector.map((value, index) => value + Number(index === 0)),
  };
  assert.throws(() => new PoseDatabase([{
    ...samples[0], id: "idle-inconsistent", features: inconsistentFeatures,
  }]), /match its structured features/);
  const otherSkeletonPose = createMotionRig().makePose();
  assert.throws(() => new PoseDatabase([
    ...samples.slice(0, 1),
    { ...samples[0], id: "idle-other-skeleton", pose: otherSkeletonPose },
  ]), /one skeleton instance/);
  const matcher = new MotionMatcher(new PoseSearch(database), {
    candidateLimit: 3,
    minimumHoldSeconds: 0,
    switchCostThreshold: 0,
  });
  assert.throws(() => matcher.update({
    pose: otherSkeletonPose,
    rootVelocity: [0, 0, 0],
    trajectory: trajectoryFor(0, 0),
  }), /database skeleton/);
  const selectedRun = matcher.update({
    pose: runPose,
    rootVelocity: [0, 0, 3],
    trajectory: trajectoryFor(0.6, 3),
    contacts: { left: true, right: false },
  });
  assert.equal(selectedRun.selectedClip, "run-forward");
  assert.equal(selectedRun.candidateCount, 3);
  assert.equal(selectedRun.transitionReason, "initial_pose_match");
  assert.equal(selectedRun.costs.total, 0);
  assert.equal(selectedRun.costs.contacts, 0);
  assert.ok(Object.values(matcher.poseSearch.normalizationScales).every((scale) => scale > 0));
  const continuedRun = matcher.update({
    pose: runPose,
    rootVelocity: [0, 0, 3],
    trajectory: trajectoryFor(0.6, 3),
    contacts: { left: true, right: false },
    dt: 0.05,
  });
  assert.equal(continuedRun.selectedClip, "run-forward");
  assert.ok(Math.abs(continuedRun.playbackTimeSeconds - 0.05) < 1e-9);
  const selectedPivot = matcher.update({
    pose: pivotPose,
    rootVelocity: [0, 0, -2],
    trajectory: trajectoryFor(-0.4, -2, 180),
    contacts: { left: false, right: true },
  });
  assert.equal(selectedPivot.selectedClip, "pivot-back");
  assert.equal(selectedPivot.transitionReason, "lower_weighted_motion_cost");

  const stableMatcher = new MotionMatcher(new PoseSearch(database), {
    candidateLimit: 2,
    minimumHoldSeconds: 0.05,
    switchCostThreshold: 0.1,
  });
  stableMatcher.update({
    pose: runPose,
    rootVelocity: [0, 0, 3],
    trajectory: trajectoryFor(0.6, 3),
    contacts: { left: true, right: false },
  });
  const held = stableMatcher.update({
    pose: pivotPose,
    rootVelocity: [0, 0, -2],
    trajectory: trajectoryFor(-0.4, -2, 180),
    contacts: { left: false, right: true },
    dt: 1 / 60,
  });
  assert.equal(held.selectedClip, "run-forward");
  assert.equal(held.transitionReason, "minimum_hold");
  let settled = held;
  for (let index = 0; index < 8; index++) {
    settled = stableMatcher.update({
      pose: pivotPose,
      rootVelocity: [0, 0, -2],
      trajectory: trajectoryFor(-0.4, -2, 180),
      contacts: { left: false, right: true },
      dt: 1 / 60,
    });
    if (settled.selectedClip === "pivot-back") break;
  }
  assert.equal(settled.selectedClip, "pivot-back");
  assert.equal(settled.transitionReason, "lower_weighted_motion_cost");
  assert.throws(() => new PoseSearch(database, { approximate: 1 }), /unknown pose search cost/);
  assert.throws(() => database.candidates[0].features.vector.push(1), TypeError);

  const history = new PoseHistory(2);
  assert.throws(() => new PoseHistory(2, ["root", "pelvis", "foot_l", "foot_r"]), /left_foot/);
  history.push(makePose(), {
    tick: 1, rootVelocity: [0, 0, 0], clipName: "history", timeSeconds: 0,
  });
  const moved = makePose({ leftFoot: [-0.2, -1, 0.1] });
  const second = history.push(moved, {
    tick: 2, rootVelocity: [0, 0, 1], clipName: "history", timeSeconds: dt, dt,
  });
  assert.deepEqual(
    second.bonePositions.left_foot,
    worldTransforms(moved)[moved.skeleton.indexByName.get("left_foot")].translation,
  );
  assert.ok(second.features.leftFootVelocity[2] > 0);
  assert.throws(() => second.bonePositions.left_foot.push(10), TypeError);
  assert.throws(() => second.features.vector.push(10), TypeError);
  assert.equal(history.latest(0).length, 0);
  assert.throws(() => history.push(moved, {
    tick: 2, rootVelocity: [0, 0, 0], clipName: "history", timeSeconds: 0.1,
  }), /monotonically/);
  assert.throws(() => history.push(moved, {
    tick: 2.5, rootVelocity: [0, 0, 0], clipName: "history", timeSeconds: 0.2,
  }), /invalid/);
  assert.throws(() => history.push(moved, {
    tick: 3, rootVelocity: [0, 0, 0], clipName: "history", timeSeconds: 0.2, dt: Infinity,
  }), /invalid/);
  const sameTimeHistory = new PoseHistory(2);
  const nearTimeA = sameTimeHistory.push(makePose(), {
    tick: 1, rootVelocity: [0, 0, 0], clipName: "same-time", timeSeconds: 0.0000001,
  });
  const nearTimeB = sameTimeHistory.push(makePose(), {
    tick: 2, rootVelocity: [0, 0, 0], clipName: "same-time", timeSeconds: 0.0000002,
  });
  assert.notEqual(nearTimeA.id, nearTimeB.id);
  history.push(makePose({ rootPosition: [0, 0, 1] }), {
    tick: 3, rootVelocity: [0, 0, 1], clipName: "history", timeSeconds: 2 * dt,
  });
  assert.deepEqual(history.latest().map(({ tick }) => tick), [2, 3]);
});
