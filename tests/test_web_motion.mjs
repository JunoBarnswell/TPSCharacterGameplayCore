import test from "node:test";
import assert from "node:assert/strict";

import {
  angleDelta,
  predictMovementStep,
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

const dt = 1 / 60;
const tuning = {
  walk_speed: 2,
  run_speed: 4.5,
  sprint_speed: 6.5,
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
    movement_mode: "grounded",
    gait: "idle",
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
    sprint: false,
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
  const history = new PredictionHistory();
  history.reset(base);
  for (const command of commands) history.predict(command, tuning, dt);
  const corrected = history.reconcile(serverStateAtAck, 1, tuning, dt);
  assert.deepEqual(corrected.position, expected.position);
  assert.deepEqual(corrected.velocity, expected.velocity);
  assert.deepEqual(history.inputs.map(({ input: pending }) => pending.sequence), [2, 3]);
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
