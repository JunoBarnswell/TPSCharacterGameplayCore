import { predictMovementStep } from "./movement-solver.mjs";

const defaultSampleTimes = [0.2, 0.4, 0.6, 0.8, 1.0];

export function rolloutTrajectory(initialState, inputIntent, tuning, dt, collisionWorld = null,
  sampleTimes = defaultSampleTimes) {
  if (!initialState || !Array.isArray(initialState.position) || initialState.position.length !== 3 ||
      !initialState.position.every(Number.isFinite) || !Array.isArray(initialState.velocity) ||
      initialState.velocity.length !== 3 || !initialState.velocity.every(Number.isFinite) ||
      !(dt > 0) || !Number.isFinite(dt) || !Array.isArray(sampleTimes) ||
      sampleTimes.length === 0 || !sampleTimes.every((time) => Number.isFinite(time) && time > 0)) {
    throw new TypeError("trajectory rollout requires a movement state, positive timestep, and sample times");
  }
  for (let index = 1; index < sampleTimes.length; index++) {
    if (sampleTimes[index] <= sampleTimes[index - 1]) {
      throw new RangeError("trajectory sample times must be strictly increasing");
    }
  }
  if (typeof inputIntent !== "function" && (!inputIntent || typeof inputIntent !== "object")) {
    throw new TypeError("trajectory rollout requires future input intent");
  }
  let state = structuredClone(initialState);
  const maximumTime = sampleTimes.at(-1);
  const steps = Math.ceil(maximumTime / dt);
  const samples = [];
  let sampleIndex = 0;
  let sequence = Number(initialState.last_processed_input ?? 0) + 1;
  for (let step = 1; step <= steps; step++) {
    const elapsed = step * dt;
    const intended = typeof inputIntent === "function" ? inputIntent(elapsed, state) : inputIntent;
    const command = {
      ...intended,
      sequence: sequence++,
      client_tick: Number(initialState.simulation_tick ?? 0) + step,
    };
    state = predictMovementStep(state, command, tuning, dt, collisionWorld);
    while (sampleIndex < sampleTimes.length && elapsed + dt * 1e-6 >= sampleTimes[sampleIndex]) {
      samples.push({
        time: sampleTimes[sampleIndex],
        sampledAt: elapsed,
        position: [...state.position],
        velocity: [...state.velocity],
        facing: Number(state.character_yaw ?? 0),
        speed: Number(state.horizontal_speed ?? 0),
        movementMode: state.movement_mode ?? "airborne",
        locomotionPhase: state.locomotion_phase ?? "idle",
      });
      sampleIndex++;
    }
  }
  if (samples.length !== sampleTimes.length) throw new Error("trajectory rollout missed a requested sample");
  return samples;
}
