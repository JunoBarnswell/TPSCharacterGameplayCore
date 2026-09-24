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
  RemoteSnapshotBuffer,
  VisualSmoothing,
} from "../aster_game/web/motion/network-motion.mjs";
import {
  createMotionFrame,
  MotionHistory,
  PoseHistory,
  predictTrajectory,
} from "../aster_game/web/motion/motion-frame.mjs";
import { evaluateAnimationGraph } from "../aster_game/web/animation/animation-graph.mjs";
import { evaluateBlendSpace } from "../aster_game/web/animation/blend-space.mjs";
import { evaluateAimOffset } from "../aster_game/web/animation/aim-offset.mjs";
import { orientationWarpAngle } from "../aster_game/web/animation/orientation-warp.mjs";
import { CollisionWorld } from "../aster_game/web/motion/collision-world.mjs";

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
    grounded: true,
    walkable_floor: true,
    ground_contact_confirmed: true,
    ground_contact_point: [0, 0, 0],
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
    action_layer: "none",
    life_state: "alive",
    jump_held: false,
    blocked_move_ticks: 0,
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

function assertVector(actual, expected, label) {
  assert.equal(actual.length, expected.length, label);
  for (let index = 0; index < expected.length; index++) {
    assert.ok(Math.abs(actual[index] - expected[index]) <= 1e-9, `${label}[${index}]`);
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
  assert.ok(history.metrics.position_error > 0);
  assert.ok(history.metrics.velocity_error > 0);
  assert.equal(history.metrics.large_correction_count, 1, JSON.stringify(history.metrics));
});

test("visual correction eases small errors and snaps large corrections", () => {
  const smoothing = new VisualSmoothing(0.12, 3);
  smoothing.correct([10, 0, 0], [9, 0, 0]);
  const first = smoothing.render([9, 0, 0], 0);
  const settled = smoothing.render([9, 0, 0], 0.12);
  assert.deepEqual(first, [10, 0, 0]);
  assert.ok(settled[0] > 9 && settled[0] < 10);
  smoothing.correct([20, 0, 0], [9, 0, 0]);
  assert.deepEqual(smoothing.render([9, 0, 0], 0), [9, 0, 0]);
});

test("remote snapshots interpolate with velocity and cap extrapolation", () => {
  const buffer = new RemoteSnapshotBuffer();
  buffer.push({ tick: 0, position: [0, 0, 0], velocity: [60, 0, 0], character_yaw: 0 });
  buffer.push({ tick: 2, position: [2, 0, 0], velocity: [60, 0, 0], character_yaw: 10 });
  assert.deepEqual(buffer.sample(1, 60).position, [1, 0, 0]);
  assert.deepEqual(buffer.sample(100, 60, 0.1).position, [8, 0, 0]);
  buffer.push({ tick: 3, position: [30, 0, 0], velocity: [0, 0, 0], character_yaw: 0 });
  assert.equal(buffer.samples.length, 1);
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
  for (let sequence = 1; sequence <= 120; sequence++) {
    state = predictMovementStep(
      state,
      input(sequence),
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
    history.predict(input(sequence), collisionTuning, dt, collisionWorld);
  }
  const acknowledged = predictMovementStep(
    baseState,
    input(1),
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
  assert.equal(history.metrics.large_correction_count, 1, JSON.stringify(history.metrics));
  assert.ok(history.metrics.position_error > 0.5);

  const alongWall = collisionWorld.moveCharacter(
    { ...state, position: [1.56, 0.9, -3], blocked_move_ticks: 0 },
    [1.56, 0.9, -2.9],
    [0, 0, 6],
    collisionTuning,
    dt,
  );
  assert.ok(alongWall.position[2] > -3);
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

test("MotionFrame, trajectory and bounded motion history carry animation inputs", () => {
  const state = { ...idleState(), velocity: [1, 0, 2], horizontal_speed: Math.sqrt(5), character_yaw: 15 };
  const frame = createMotionFrame(state, 100, dt);
  assert.equal(frame.tick, 100);
  assert.equal(frame.footIK.leftFootGroundDistance, null);
  assert.ok(Math.abs(frame.movementDirection - (Math.atan2(1, 2) * 180 / Math.PI - 15)) < 1e-9);
  assert.deepEqual(predictTrajectory(frame).map((sample) => sample.time), [0.2, 0.4, 0.6, 0.8, 1]);
  const motionHistory = new MotionHistory(2);
  const poseHistory = new PoseHistory(1);
  motionHistory.push(frame);
  motionHistory.push(frame);
  motionHistory.push(frame);
  poseHistory.push(frame);
  assert.equal(motionHistory.latest().length, 2);
  assert.equal(poseHistory.latest().length, 1);
});

test("animation graph returns normalized blend, additive aim, and warp semantics", () => {
  const frame = createMotionFrame({
    ...idleState(),
    horizontal_speed: 4.5,
    velocity: [0, 0, 4.5],
    aim_yaw: 220,
    aim_pitch: -95,
    action_layer: "hit_reaction",
    hit_strength: 0.6,
  }, 10);
  const weights = evaluateBlendSpace(frame, tuning.sprint_speed);
  assert.ok(Math.abs(Object.values(weights).reduce((sum, value) => sum + value, 0) - 1) < 1e-9);
  assert.deepEqual(evaluateAimOffset(220, -95), { yaw: 180, pitch: -89 });
  assert.equal(Math.abs(orientationWarpAngle(180, 0)), 90);
  const graph = evaluateAnimationGraph(frame, tuning, new Map(), dt);
  assert.equal(graph.hitReaction, 0.6);
  assert.equal(graph.phase, "idle");
});
