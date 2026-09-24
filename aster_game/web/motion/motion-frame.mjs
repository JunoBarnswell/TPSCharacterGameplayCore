import { angleDelta } from "./movement-solver.mjs";

export function createMotionFrame(state, tick, renderDelta = 0) {
  const worldMovementDirection = Math.atan2(
    state.velocity?.[0] ?? 0,
    state.velocity?.[2] ?? 0,
  ) * 180 / Math.PI;
  const characterYaw = Number(state.character_yaw ?? 0);
  const viewYaw = Number(state.view_yaw ?? state.desired_facing_yaw ?? characterYaw);
  const turnRemaining = state.locomotion_phase === "turn_in_place"
    ? Number(state.remaining_turn_angle ?? angleDelta(viewYaw, characterYaw))
    : 0;
  const turnAngle = Math.max(1, Number(state.turn_angle ?? Math.abs(turnRemaining)));
  const phaseDuration = Number(state.phase_duration_ticks ?? 0);
  const derivedPhaseProgress = phaseDuration > 0
    ? Math.max(0, Math.min(1, (tick - Number(state.phase_start_tick ?? tick)) / phaseDuration))
    : 0;
  return Object.freeze({
    tick,
    position: [...(state.position ?? [0, 0, 0])],
    speed: Number(state.current_speed ?? 0),
    horizontalSpeed: Number(state.horizontal_speed ?? 0),
    verticalSpeed: Number(state.vertical_speed ?? 0),
    velocity: [...(state.velocity ?? [0, 0, 0])],
    acceleration: [...(state.acceleration ?? [0, 0, 0])],
    desiredVelocity: [...(state.desired_velocity ?? [0, 0, 0])],
    movementDirection: angleDelta(
      worldMovementDirection,
      Number(state.character_yaw ?? 0),
    ),
    worldMovementDirection,
    characterYaw,
    desiredFacingYaw: Number(state.desired_facing_yaw ?? 0),
    yawRate: Number(state.yaw_rate ?? 0),
    turnAngle: Number(state.turn_angle ?? 0),
    grounded: Boolean(state.grounded),
    floorNormal: [...(state.floor_normal ?? [0, 1, 0])],
    floorDistance: state.floor_distance ?? null,
    movementMode: state.movement_mode ?? "airborne",
    requestedGait: state.requested_gait ?? "run",
    actualGait: state.actual_gait ?? "idle",
    locomotionPhase: state.locomotion_phase ?? "idle",
    phaseProgress: Math.max(0, Math.min(1, Number(state.phase_progress ?? derivedPhaseProgress))),
    turnDirection: state.turn_direction ?? (turnRemaining > 0 ? "right" : turnRemaining < 0 ? "left" : "none"),
    turnProgress: Math.max(0, Math.min(1, Number(
      state.turn_progress ?? (state.locomotion_phase === "turn_in_place"
        ? 1 - Math.abs(turnRemaining) / turnAngle
        : 0),
    ))),
    remainingTurnAngle: turnRemaining,
    rotationMode: state.rotation_mode ?? "orient_to_movement",
    aimYaw: Number(state.aim_yaw ?? 0),
    aimPitch: Number(state.aim_pitch ?? 0),
    actionLayer: state.action_layer ?? "none",
    actionLayers: [...(state.action_layers ?? (state.action_layer && state.action_layer !== "none"
      ? [state.action_layer]
      : []))],
    hitDirection: state.hit_direction ? [...state.hit_direction] : null,
    hitStrength: Number(state.hit_strength ?? 0),
    footIK: {
      leftFootGroundDistance: null,
      rightFootGroundDistance: null,
      leftFootNormal: null,
      rightFootNormal: null,
      pelvisOffset: null,
    },
    renderDelta,
  });
}

export class MotionHistory {
  constructor(limit = 120) {
    this.limit = limit;
    this.frames = [];
  }

  push(frame) {
    this.frames.push(frame);
    if (this.frames.length > this.limit) this.frames.splice(0, this.frames.length - this.limit);
  }

  latest(count = this.frames.length) {
    return this.frames.slice(-count);
  }
}

export class PoseHistory extends MotionHistory {}

export function predictTrajectory(frame, seconds = [0.2, 0.4, 0.6, 0.8, 1.0]) {
  return seconds.map((time) => ({
    time,
    position: frame.velocity.map((velocity, axis) =>
      frame.position[axis] + velocity * time + 0.5 * frame.acceleration[axis] * time * time),
    facing: frame.characterYaw + frame.yawRate * time,
    velocity: frame.velocity.map((velocity, axis) => velocity + frame.acceleration[axis] * time),
  }));
}
