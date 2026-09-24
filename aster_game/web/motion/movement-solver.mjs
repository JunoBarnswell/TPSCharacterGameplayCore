export function normalizeDegrees(angle) {
  return ((angle + 180) % 360 + 360) % 360 - 180;
}

export function angleDelta(target, current) {
  return normalizeDegrees(target - current);
}

export function evaluateResponseCurve(points, value) {
  if (!Array.isArray(points) || points.length < 2) {
    throw new RangeError("response curves require at least two points");
  }
  const x = Math.max(0, Math.min(1, value));
  if (x <= points[0][0]) return points[0][1];
  for (let index = 0; index < points.length - 1; index++) {
    const [x0, y0] = points[index];
    const [x1, y1] = points[index + 1];
    if (x <= x1) {
      const alpha = (x - x0) / (x1 - x0);
      return y0 + (y1 - y0) * alpha;
    }
  }
  return points.at(-1)[1];
}

const defaultAccelerationCurve = [[0, 1.35], [0.5, 1], [1, 0.65]];
const defaultBrakingCurve = [[0, 0.6], [0.35, 1], [1, 1.35]];
const defaultTurnSpeedCurve = [[0, 0.22], [0.25, 0.55], [1, 1]];

export function desiredMotion(input, tuning) {
  const axisLength = Math.hypot(input.move_x, input.move_z);
  if (axisLength <= 1e-4) {
    return { velocity: [0, 0, 0], direction: [0, 0, 0], requestedGait: input.requested_gait };
  }
  const scale = Math.min(1, axisLength) / axisLength;
  const localX = input.move_x * scale;
  const localZ = input.move_z * scale;
  const speeds = {
    walk: tuning.walk_speed,
    run: tuning.run_speed,
    sprint: tuning.sprint_speed,
  };
  const speed = speeds[input.requested_gait];
  if (speed === undefined) throw new RangeError(`unknown requested gait: ${input.requested_gait}`);
  const angle = input.view_yaw * Math.PI / 180;
  const x = localX * Math.cos(angle) + localZ * Math.sin(angle);
  const z = -localX * Math.sin(angle) + localZ * Math.cos(angle);
  const magnitude = Math.hypot(x, z);
  return {
    velocity: [x * speed, 0, z * speed],
    direction: [x / magnitude, 0, z / magnitude],
    requestedGait: input.requested_gait,
  };
}

function moveTowards(current, target, maxDelta) {
  const dx = target[0] - current[0];
  const dz = target[1] - current[1];
  const distance = Math.hypot(dx, dz);
  if (distance <= maxDelta || distance <= 1e-12) return target;
  const scale = maxDelta / distance;
  return [current[0] + dx * scale, current[1] + dz * scale];
}

export function deriveActualGait(horizontalSpeed, tuning) {
  if (horizontalSpeed <= 0.1) return "idle";
  if (horizontalSpeed < (tuning.walk_speed + tuning.run_speed) / 2) return "walk";
  if (horizontalSpeed < (tuning.run_speed + tuning.sprint_speed) / 2) return "run";
  return "sprint";
}

export function projectVelocityOntoGroundPlane(velocity, normal) {
  const normalLength = Math.hypot(...normal);
  if (normalLength <= 1e-8) throw new RangeError("ground normal must have non-zero length");
  const unitNormal = normal.map((component) => component / normalLength);
  const normalVelocity = velocity.reduce((sum, value, axis) => sum + value * unitNormal[axis], 0);
  const projected = velocity.map((value, axis) => value - normalVelocity * unitNormal[axis]);
  const originalSpeed = Math.hypot(...velocity);
  const projectedSpeed = Math.hypot(...projected);
  if (projectedSpeed <= 1e-8 || originalSpeed <= 1e-8) return [0, 0, 0];
  const scale = originalSpeed / projectedSpeed;
  return projected.map((component) => component * scale);
}

export function solveHorizontalVelocity(current, desired, dt, grounded, tuning, requestedGait = "run") {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  let vx = current[0];
  let vz = current[2];
  const dx = desired[0];
  const dz = desired[2];
  const desiredSpeed = Math.hypot(dx, dz);
  if (grounded) {
    if (desiredSpeed <= 1e-6) {
      const speed = Math.hypot(vx, vz);
      const brakingCurve = tuning.braking_curve ?? defaultBrakingCurve;
      const response = evaluateResponseCurve(brakingCurve, speed / tuning.sprint_speed);
      const deceleration = tuning.braking_deceleration * response + tuning.ground_friction * speed;
      const amount = Math.min(speed, deceleration * dt);
      const factor = speed ? (speed - amount) / speed : 0;
      vx *= factor;
      vz *= factor;
    } else {
      const directionX = dx / desiredSpeed;
      const directionZ = dz / desiredSpeed;
      const projection = vx * directionX + vz * directionZ;
      const lateralX = vx - projection * directionX;
      const lateralZ = vz - projection * directionZ;
      const lateralSpeed = Math.hypot(lateralX, lateralZ);
      const lateralReduction = Math.min(
        lateralSpeed,
        (tuning.ground_directional_friction ?? 9) * dt,
      );
      const lateralScale = lateralSpeed ? (lateralSpeed - lateralReduction) / lateralSpeed : 0;
      vx = projection * directionX + lateralX * lateralScale;
      vz = projection * directionZ + lateralZ * lateralScale;

      const currentSpeed = Math.hypot(vx, vz);
      if (currentSpeed > 1e-6) {
        const dot = Math.max(-1, Math.min(1, (vx * directionX + vz * directionZ) / currentSpeed));
        const turnFraction = (1 - dot) * 0.5;
        let turnDeceleration = (tuning.turning_deceleration ?? 7) * turnFraction;
        const pivotThreshold = tuning.pivot_angle_threshold ?? 135;
        if (dot <= Math.cos(pivotThreshold * Math.PI / 180)) {
          turnDeceleration *= tuning.pivot_braking_multiplier ?? 1.75;
        }
        const braking = Math.min(currentSpeed, turnDeceleration * dt);
        const turnScale = (currentSpeed - braking) / currentSpeed;
        vx *= turnScale;
        vz *= turnScale;
      }

      const accelerationCurve = tuning[`${requestedGait}_acceleration_curve`] ?? defaultAccelerationCurve;
      const speedFraction = Math.min(1, Math.hypot(vx, vz) / tuning.sprint_speed);
      const response = evaluateResponseCurve(accelerationCurve, speedFraction);
      const maxChange = tuning.ground_acceleration * response * dt;
      [vx, vz] = moveTowards([vx, vz], [dx, dz], maxChange);
    }
  } else {
    const changeX = dx - vx;
    const changeZ = dz - vz;
    const changeLength = Math.hypot(changeX, changeZ);
    const maxChange = tuning.air_acceleration * tuning.air_control * dt;
    const factor = changeLength ? Math.min(1, maxChange / changeLength) : 1;
    vx += changeX * factor;
    vz += changeZ * factor;
    const speed = Math.hypot(vx, vz);
    if (speed > tuning.air_max_speed) {
      vx *= tuning.air_max_speed / speed;
      vz *= tuning.air_max_speed / speed;
    }
  }
  return {
    velocity: [vx, current[1], vz],
    acceleration: [(vx - current[0]) / dt, 0, (vz - current[2]) / dt],
  };
}

export function solveRotation(currentYaw, angularVelocity, desiredYaw, dt, tuning) {
  if (!(dt > 0)) throw new RangeError("dt must be positive");
  const difference = angleDelta(desiredYaw, currentYaw);
  const turnFraction = Math.min(1, Math.abs(difference) / 180);
  const effectiveMaxSpeed = tuning.max_rotation_speed * evaluateResponseCurve(
    tuning.turn_speed_curve ?? defaultTurnSpeedCurve,
    turnFraction,
  );
  const brakeRate = Math.sqrt(2 * tuning.rotation_acceleration * Math.abs(difference));
  const targetRate = Math.abs(difference) <= 1e-5
    ? 0
    : Math.sign(difference) * Math.min(effectiveMaxSpeed, brakeRate);
  const rateLimit = angularVelocity * targetRate >= 0
    ? tuning.rotation_acceleration
    : tuning.rotation_deceleration;
  const deltaRate = Math.max(-rateLimit * dt, Math.min(rateLimit * dt, targetRate - angularVelocity));
  const nextRate = angularVelocity + deltaRate;
  const applied = nextRate * dt;
  if (Math.abs(applied) > Math.abs(difference) && nextRate * difference > 0) {
    return { yaw: normalizeDegrees(desiredYaw), angularVelocity: 0 };
  }
  return { yaw: normalizeDegrees(currentYaw + applied), angularVelocity: nextRate };
}

export function predictMovementStep(state, input, tuning, dt, collisionWorld = null) {
  if (state.life_state === "dead") return { ...state, last_processed_input: input.sequence };
  const desired = desiredMotion(input, tuning);
  const grounded = state.grounded && state.walkable_floor !== false;
  if (grounded && (state.floor_normal?.[1] ?? 1) > 1e-4) {
    desired.velocity = projectVelocityOntoGroundPlane(
      desired.velocity,
      state.floor_normal ?? [0, 1, 0],
    );
    const projectedLength = Math.hypot(...desired.velocity);
    desired.direction = projectedLength > 1e-6
      ? desired.velocity.map((component) => component / projectedLength)
      : [0, 0, 0];
  }
  const desiredSpeed = Math.hypot(desired.velocity[0], desired.velocity[2]);
  const solved = solveHorizontalVelocity(
    state.velocity,
    desired.velocity,
    dt,
    grounded,
    tuning,
    desired.requestedGait,
  );
  const simulationTick = Number(state.simulation_tick ?? state.server_tick ?? 0);
  const jumpPressed = input.jump && !state.jump_held && grounded &&
    simulationTick >= Number(state.jump_available_tick ?? 0);
  let vy = state.velocity[1];
  let y = state.position[1];
  let nextGrounded = grounded;
  let phase = state.locomotion_phase;
  let phaseUntilTick = Number(state.phase_until_tick ?? 0);
  let phaseStartTick = Number(state.phase_start_tick ?? simulationTick);
  let phaseDurationTicks = Number(state.phase_duration_ticks ?? 0);
  let landingRecoveryUntilTick = Number(state.landing_recovery_until_tick ?? 0);
  let turnAngle = Number(state.turn_angle ?? 0);
  let turnDirection = state.turn_direction ?? "none";
  const setPhase = (next, durationTicks = 0) => {
    if (next !== phase || (durationTicks > 0 && simulationTick >= phaseUntilTick)) {
      phaseStartTick = simulationTick;
      phaseDurationTicks = durationTicks;
    }
    phase = next;
    if (durationTicks > 0) phaseUntilTick = simulationTick + durationTicks;
  };
  const previousHorizontalSpeed = Math.hypot(state.velocity[0], state.velocity[2]);
  const previousDesired = state.desired_velocity ?? [0, 0, 0];
  if (jumpPressed) {
    vy = tuning.jump_speed;
    nextGrounded = false;
    setPhase("jump_start", Math.max(1, Math.ceil(0.2 / dt)));
    state = {
      ...state,
      jump_available_tick: simulationTick + Math.ceil(tuning.jump_cooldown_seconds / dt),
    };
  }
  if (!nextGrounded) {
    vy = Math.max(-tuning.max_fall_speed, vy - tuning.gravity * dt);
    y += vy * dt;
    const apexThreshold = tuning.apex_velocity_threshold;
    if (!jumpPressed && phase === "jump_start" && vy > apexThreshold) setPhase("rising");
    else if (phase === "rising" && vy <= apexThreshold) setPhase("apex");
    else if (phase === "apex" && vy < -apexThreshold) setPhase("falling");
    else if (!state.grounded && vy <= -apexThreshold && !["jump_start", "rising"].includes(phase)) setPhase("falling");
  } else {
    const normal = state.floor_normal ?? [0, 1, 0];
    vy = normal[1] > 1e-4
      ? -(normal[0] * solved.velocity[0] + normal[2] * solved.velocity[2]) / normal[1]
      : 0;
    y += vy * dt;
  }
  const position = [
    state.position[0] + solved.velocity[0] * dt,
    y,
    state.position[2] + solved.velocity[2] * dt,
  ];
  let velocity = [solved.velocity[0], vy, solved.velocity[2]];
  let collision = null;
  if (collisionWorld) {
    collision = collisionWorld.moveCharacter(
      { ...state, grounded: grounded && !jumpPressed },
      position,
      velocity,
      tuning,
      dt,
    );
    position.splice(0, 3, ...collision.position);
    velocity = collision.velocity;
    nextGrounded = collision.grounded;
  }
  if ((collision?.landed ?? false) || (state.movement_mode === "airborne" && nextGrounded)) {
    const impact = Math.max(Math.abs(state.vertical_speed ?? 0), state.landing_impact_velocity ?? 0);
    setPhase(impact < tuning.landing_soft_velocity
      ? "soft_land"
      : impact >= tuning.landing_heavy_velocity ? "heavy_land" : "normal_land",
    Math.max(1, Math.ceil(tuning.landing_recovery_seconds / dt)));
    landingRecoveryUntilTick = simulationTick + Math.max(1, Math.ceil(tuning.landing_recovery_seconds / dt));
    phaseUntilTick = landingRecoveryUntilTick;
  } else if (nextGrounded) {
    const landingPhases = ["soft_land", "normal_land", "heavy_land"];
    const recoveringLanding = landingPhases.includes(phase) && simulationTick < landingRecoveryUntilTick;
    if (recoveringLanding) {
      // Preserve the impact pose semantic while the server recovery window is active.
    } else if (landingPhases.includes(phase)) {
      setPhase("idle");
    }
    const transitions = ["start", "stop", "pivot", "turn_in_place"];
    if (transitions.includes(phase) && simulationTick < phaseUntilTick) {
      if (phase !== "turn_in_place" || Math.abs(angleDelta(input.view_yaw, state.character_yaw)) >= 3) {
        // Keep the transition channel through its authoritative time window.
      } else {
        phaseUntilTick = simulationTick;
      }
    }
    if (!recoveringLanding && (!transitions.includes(phase) || simulationTick >= phaseUntilTick)) {
      const oldSpeed = previousHorizontalSpeed;
      if (desiredSpeed > 0.1) {
        if (oldSpeed < 0.25) {
          setPhase("start", Math.max(1, Math.ceil(0.2 / dt)));
        } else {
          const oldX = previousDesired[0] ?? 0;
          const oldZ = previousDesired[2] ?? 0;
          const oldDesiredSpeed = Math.hypot(oldX, oldZ);
          const dot = (oldX * desired.velocity[0] + oldZ * desired.velocity[2]) /
            Math.max(1e-6, oldDesiredSpeed * desiredSpeed);
          const pivotDotThreshold = Math.cos(tuning.pivot_angle_threshold * Math.PI / 180);
          if (
            oldSpeed > 1 &&
            Math.hypot(solved.velocity[0], solved.velocity[2]) > 1 &&
            dot <= pivotDotThreshold
          ) {
            setPhase("pivot", Math.max(1, Math.ceil(0.2 / dt)));
          } else {
            setPhase("loop");
          }
        }
      } else if (oldSpeed > 0.5) {
        setPhase("stop", Math.max(1, Math.ceil(0.2 / dt)));
      } else if (Math.abs(angleDelta(input.view_yaw, state.character_yaw)) >= tuning.turn_in_place_threshold) {
        turnAngle = Math.min(180, Math.max(45, Math.round(Math.abs(angleDelta(input.view_yaw, state.character_yaw)) / 45) * 45));
        turnDirection = angleDelta(input.view_yaw, state.character_yaw) > 0 ? "right" : "left";
        setPhase("turn_in_place", Math.max(1, Math.ceil(0.25 / dt)));
      } else {
        setPhase("idle");
      }
    }
  }
  let desiredFacing = state.desired_facing_yaw;
  if (input.rotation_mode === "aim" || input.rotation_mode === "strafe") {
    desiredFacing = input.view_yaw;
  } else if (desiredSpeed > 1e-4) {
    desiredFacing = Math.atan2(desired.velocity[0], desired.velocity[2]) * 180 / Math.PI;
  } else if (Math.abs(angleDelta(input.view_yaw, state.character_yaw)) >= tuning.turn_in_place_threshold) {
    desiredFacing = input.view_yaw;
  }
  const rotation = solveRotation(state.character_yaw, state.angular_velocity, desiredFacing, dt, tuning);
  const turnRemaining = phase === "turn_in_place" ? angleDelta(desiredFacing, rotation.yaw) : 0;
  const phaseProgress = phaseDurationTicks > 0
    ? Math.max(0, Math.min(1, (simulationTick - phaseStartTick) / phaseDurationTicks))
    : 0;
  return {
    ...state,
    position,
    velocity,
    acceleration: [
      solved.acceleration[0],
      (velocity[1] - state.velocity[1]) / dt,
      solved.acceleration[2],
    ],
    desired_velocity: desired.velocity,
    desired_move_direction: desired.direction,
    current_speed: Math.hypot(Math.hypot(velocity[0], velocity[2]), velocity[1]),
    horizontal_speed: Math.hypot(velocity[0], velocity[2]),
    vertical_speed: vy,
    character_yaw: rotation.yaw,
    desired_facing_yaw: normalizeDegrees(desiredFacing),
    angular_velocity: rotation.angularVelocity,
    yaw_rate: rotation.angularVelocity,
    view_yaw: input.view_yaw,
    view_pitch: input.view_pitch,
    aim_yaw: angleDelta(input.view_yaw, rotation.yaw),
    aim_pitch: input.view_pitch,
    movement_mode: nextGrounded ? "grounded" : "airborne",
    grounded: nextGrounded,
    floor_normal: collision?.floor_normal ?? state.floor_normal ?? [0, 1, 0],
    floor_distance: collision?.floor_distance ?? state.floor_distance ?? null,
    walkable_floor: collision?.walkable_floor ?? state.walkable_floor ?? true,
    ground_contact_confirmed: collision?.ground_contact_confirmed ??
      state.ground_contact_confirmed ?? nextGrounded,
    ground_contact_point: collision?.ground_contact_point ?? state.ground_contact_point ?? null,
    ground_entity: collision?.ground_entity ?? state.ground_entity ?? null,
    ground_sample_count: collision?.ground_sample_count ?? state.ground_sample_count ?? 0,
    slope_angle: collision?.slope_angle ?? state.slope_angle ?? 0,
    blocked_move_ticks: collision?.blocked_move_ticks ?? state.blocked_move_ticks ?? 0,
    step_up: Boolean(collision?.step_up),
    actual_gait: deriveActualGait(Math.hypot(solved.velocity[0], solved.velocity[2]), tuning),
    requested_gait: desired.requestedGait,
    rotation_mode: input.rotation_mode,
    locomotion_phase: phase,
    phase_start_tick: phaseStartTick,
    phase_duration_ticks: phaseDurationTicks,
    phase_progress: phaseProgress,
    phase_until_tick: phaseUntilTick,
    landing_recovery_until_tick: landingRecoveryUntilTick,
    turn_angle: turnAngle,
    turn_direction: phase === "turn_in_place"
      ? turnDirection
      : turnAngle > 0 ? turnDirection : "none",
    remaining_turn_angle: turnRemaining,
    turn_progress: phase === "turn_in_place"
      ? Math.max(0, Math.min(1, 1 - Math.abs(turnRemaining) / Math.max(1, turnAngle)))
      : 0,
    jump_held: input.jump,
    last_processed_input: input.sequence,
    simulation_tick: simulationTick + 1,
  };
}
