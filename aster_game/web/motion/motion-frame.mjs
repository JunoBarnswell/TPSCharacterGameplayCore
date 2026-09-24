import { angleDelta } from "./movement-solver.mjs";

export function createMotionFrame(state, tick, renderDelta = 0) {
  const worldMovementDirection = Math.atan2(
    state.velocity?.[0] ?? 0,
    state.velocity?.[2] ?? 0,
  ) * 180 / Math.PI;
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
    characterYaw: Number(state.character_yaw ?? 0),
    desiredFacingYaw: Number(state.desired_facing_yaw ?? 0),
    yawRate: Number(state.yaw_rate ?? 0),
    turnAngle: Number(state.turn_angle ?? 0),
    grounded: Boolean(state.grounded),
    floorNormal: [...(state.floor_normal ?? [0, 1, 0])],
    floorDistance: state.floor_distance ?? null,
    movementMode: state.movement_mode ?? "airborne",
    gait: state.gait ?? "idle",
    locomotionPhase: state.locomotion_phase ?? "idle",
    rotationMode: state.rotation_mode ?? "orient_to_movement",
    aimYaw: Number(state.aim_yaw ?? 0),
    aimPitch: Number(state.aim_pitch ?? 0),
    actionLayer: state.action_layer ?? "none",
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
