import { angleDelta, predictMovementStep } from "./movement-solver.mjs";

export class FixedStepScheduler {
  constructor(tickRate = 60, maxCatchUpSteps = 4) {
    if (!Number.isFinite(tickRate) || tickRate <= 0 ||
        !Number.isInteger(maxCatchUpSteps) || maxCatchUpSteps < 1) {
      throw new RangeError("fixed-step scheduler requires a positive rate and catch-up limit");
    }
    this.maxCatchUpSteps = maxCatchUpSteps;
    this.setTickRate(tickRate);
    this.reset();
  }

  setTickRate(tickRate) {
    if (!Number.isFinite(tickRate) || tickRate <= 0) {
      throw new RangeError("fixed-step scheduler tick rate must be positive");
    }
    this.stepMilliseconds = 1000 / tickRate;
    this.accumulatorMilliseconds = 0;
    this.lastTimeMilliseconds = null;
  }

  reset(nowMilliseconds = null) {
    if (nowMilliseconds !== null && !Number.isFinite(nowMilliseconds)) {
      throw new TypeError("fixed-step scheduler reset time must be finite");
    }
    this.accumulatorMilliseconds = 0;
    this.lastTimeMilliseconds = nowMilliseconds;
  }

  advance(nowMilliseconds) {
    if (!Number.isFinite(nowMilliseconds)) {
      throw new TypeError("fixed-step scheduler time must be finite");
    }
    if (this.lastTimeMilliseconds === null || nowMilliseconds < this.lastTimeMilliseconds) {
      this.reset(nowMilliseconds);
      return [];
    }
    const elapsed = nowMilliseconds - this.lastTimeMilliseconds;
    this.lastTimeMilliseconds = nowMilliseconds;
    this.accumulatorMilliseconds = Math.min(
      this.accumulatorMilliseconds + elapsed,
      this.stepMilliseconds * this.maxCatchUpSteps,
    );
    const scheduledTimes = [];
    while (this.accumulatorMilliseconds + 1e-6 >= this.stepMilliseconds &&
        scheduledTimes.length < this.maxCatchUpSteps) {
      this.accumulatorMilliseconds -= this.stepMilliseconds;
      scheduledTimes.push(nowMilliseconds - this.accumulatorMilliseconds);
    }
    return scheduledTimes;
  }
}

export class PredictionHistory {
  constructor(limit = 256, largeCorrectionThreshold = 0.5) {
    if (!Number.isInteger(limit) || limit < 1 || !Number.isFinite(largeCorrectionThreshold) ||
        largeCorrectionThreshold < 0) {
      throw new RangeError("prediction history requires a positive limit and correction threshold");
    }
    this.limit = limit;
    this.largeCorrectionThreshold = largeCorrectionThreshold;
    this.state = null;
    this.baseState = null;
    this.baseSequence = null;
    this.inputs = [];
    this.metrics = {
      position_error: 0,
      rotation_error: 0,
      velocity_error: 0,
      reconciliation_count: 0,
      large_correction_count: 0,
      correction_position_error: 0,
      history_overflow_count: 0,
      hard_resync_count: 0,
      discarded_input_count: 0,
      stale_ack_count: 0,
      last_resync_reason: null,
    };
  }

  reset(authoritative) {
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    this.baseState = structuredClone(this.state);
    this.baseSequence = Number(authoritative.last_processed_input ?? -1);
    this.inputs = [];
    this.metrics = {
      position_error: 0,
      rotation_error: 0,
      velocity_error: 0,
      reconciliation_count: 0,
      large_correction_count: 0,
      correction_position_error: 0,
      history_overflow_count: 0,
      hard_resync_count: 0,
      discarded_input_count: 0,
      stale_ack_count: 0,
      last_resync_reason: null,
    };
    this.lastAckSequence = Number(authoritative.last_processed_input ?? -1);
  }

  predict(input, tuning, dt, collisionWorld = null) {
    if (!this.state) return null;
    if (!input || !Number.isSafeInteger(input.sequence) || input.sequence < 0) {
      throw new TypeError("predicted input requires a non-negative safe sequence number");
    }
    const previousSequence = this.inputs.at(-1)?.input.sequence ?? this.baseSequence;
    if (previousSequence !== null && input.sequence <= previousSequence) {
      throw new RangeError("predicted input sequences must increase strictly");
    }
    this.state = predictMovementStep(this.state, input, tuning, dt, collisionWorld);
    this.inputs.push({ input: { ...input }, state: structuredClone(this.state), sentAt: input.sentAt });
    if (this.inputs.length > this.limit) {
      const evicted = this.inputs.shift();
      this.baseState = evicted.state;
      this.baseSequence = Number(evicted.input.sequence);
      this.metrics.history_overflow_count++;
    }
    return this.state;
  }

  reconcile(authoritative, ack, tuning, dt, collisionWorld = null) {
    if (!Number.isSafeInteger(ack) || ack < -1 || !authoritative ||
        !Array.isArray(authoritative.position) || authoritative.position.length !== 3 ||
        !authoritative.position.every(Number.isFinite) || !Array.isArray(authoritative.velocity) ||
        authoritative.velocity.length !== 3 || !authoritative.velocity.every(Number.isFinite)) {
      throw new TypeError("prediction reconciliation requires a valid ACK and authoritative transform");
    }
    if (!this.state || !authoritative) {
      this.reset(authoritative);
      return this.state;
    }
    if (ack < this.lastAckSequence) {
      this.metrics.stale_ack_count++;
      return this.state;
    }
    const latestPredictedSequence = this.inputs.at(-1)?.input.sequence ?? this.baseSequence;
    if (latestPredictedSequence !== null && ack > latestPredictedSequence) {
      throw new RangeError("authoritative ACK cannot exceed the latest predicted input sequence");
    }
    this.metrics.reconciliation_count++;
    const predictedAtAck = ack === this.baseSequence
      ? this.baseState
      : this.inputs.find(({ input }) => input.sequence === ack)?.state ?? null;
    if (predictedAtAck) {
      this.metrics.position_error = Math.hypot(
        ...predictedAtAck.position.map((value, axis) => value - authoritative.position[axis]),
      );
      this.metrics.rotation_error = Math.abs(
        angleDelta(predictedAtAck.character_yaw, authoritative.character_yaw),
      );
      this.metrics.velocity_error = Math.hypot(
        ...predictedAtAck.velocity.map((value, axis) => value - authoritative.velocity[axis]),
      );
    } else {
      this.metrics.position_error = null;
      this.metrics.rotation_error = null;
      this.metrics.velocity_error = null;
    }

    const beforeReplay = this.state;
    const historyHasGap = predictedAtAck === null || ack < this.baseSequence;
    if (historyHasGap) {
      this.metrics.hard_resync_count++;
      this.metrics.last_resync_reason = ack < this.baseSequence
        ? "prediction_history_overflow"
        : "ack_state_unavailable";
      this.metrics.discarded_input_count += Math.max(0, this.baseSequence - ack);
      this.metrics.discarded_input_count += this.inputs.filter(
        ({ input }) => input.sequence > ack,
      ).length;
      this.inputs = [];
      this.state = {
        ...authoritative,
        position: [...authoritative.position],
        velocity: [...authoritative.velocity],
      };
      this.baseState = structuredClone(this.state);
      this.baseSequence = ack;
      this.lastAckSequence = ack;
      this.metrics.correction_position_error = Math.hypot(
        ...beforeReplay.position.map((value, axis) => value - this.state.position[axis]),
      );
      if (this.metrics.correction_position_error > this.largeCorrectionThreshold) {
        this.metrics.large_correction_count++;
      }
      return this.state;
    }

    this.inputs = this.inputs.filter(({ input }) => input.sequence > ack);
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    this.baseState = structuredClone(this.state);
    this.baseSequence = ack;
    this.lastAckSequence = ack;
    for (const entry of this.inputs) {
      this.state = predictMovementStep(this.state, entry.input, tuning, dt, collisionWorld);
      entry.state = structuredClone(this.state);
    }
    this.metrics.correction_position_error = Math.hypot(
      ...beforeReplay.position.map((value, axis) => value - this.state.position[axis]),
    );
    if (this.metrics.correction_position_error > this.largeCorrectionThreshold) {
      this.metrics.large_correction_count++;
    }
    return this.state;
  }
}

function normalizeDegrees(degrees) {
  return angleDelta(degrees, 0);
}

export class VisualTransformSmoothing {
  constructor({ durationSeconds = 0.12, snapDistance = 3, snapYawDegrees = 120, mode = "exponential" } = {}) {
    if (!(durationSeconds > 0) || !(snapDistance > 0) || !(snapYawDegrees > 0)) {
      throw new RangeError("smoothing duration and snap limits must be positive");
    }
    if (!["exponential", "linear", "snap"].includes(mode)) {
      throw new TypeError("visual smoothing mode must be exponential, linear, or snap");
    }
    this.durationSeconds = durationSeconds;
    this.snapDistance = snapDistance;
    this.snapYawDegrees = snapYawDegrees;
    this.mode = mode;
    this.positionOffset = [0, 0, 0];
    this.yawOffset = 0;
    this.lastCorrection = "none";
  }

  reset() {
    this.positionOffset = [0, 0, 0];
    this.yawOffset = 0;
    this.lastCorrection = "none";
  }

  correct(previousVisual, simulationTransform) {
    if (!previousVisual || !simulationTransform || previousVisual.position.length !== 3 ||
        simulationTransform.position.length !== 3) {
      throw new TypeError("visual correction requires position and yaw transforms");
    }
    const positionError = previousVisual.position.map(
      (value, index) => value - simulationTransform.position[index],
    );
    const yawError = angleDelta(
      Number(previousVisual.character_yaw ?? 0),
      Number(simulationTransform.character_yaw ?? 0),
    );
    if (this.mode === "snap" || Math.hypot(...positionError) > this.snapDistance ||
        Math.abs(yawError) > this.snapYawDegrees) {
      this.reset();
      this.lastCorrection = "snap";
      return this.lastCorrection;
    }
    this.positionOffset = positionError;
    this.yawOffset = yawError;
    this.lastCorrection = "smooth";
    return this.lastCorrection;
  }

  render(simulationTransform, dt) {
    const elapsed = Math.max(0, dt);
    const decay = this.mode === "snap"
      ? 0
      : this.mode === "linear"
        ? Math.max(0, 1 - elapsed / this.durationSeconds)
        : Math.exp(-elapsed / this.durationSeconds);
    this.positionOffset = this.positionOffset.map((value) => value * decay);
    this.yawOffset *= decay;
    return {
      position: simulationTransform.position.map((value, index) => value + this.positionOffset[index]),
      character_yaw: normalizeDegrees(Number(simulationTransform.character_yaw ?? 0) + this.yawOffset),
    };
  }
}

function hermite(p0, v0, p1, v1, t, duration) {
  const t2 = t * t;
  const t3 = t2 * t;
  const h00 = 2 * t3 - 3 * t2 + 1;
  const h10 = t3 - 2 * t2 + t;
  const h01 = -2 * t3 + 3 * t2;
  const h11 = t3 - t2;
  return p0.map((value, index) =>
    h00 * value + h10 * duration * v0[index] + h01 * p1[index] + h11 * duration * v1[index]);
}

function hermiteVelocity(p0, v0, p1, v1, t, duration) {
  const t2 = t * t;
  const dh00 = 6 * t2 - 6 * t;
  const dh10 = 3 * t2 - 4 * t + 1;
  const dh01 = -6 * t2 + 6 * t;
  const dh11 = 3 * t2 - 2 * t;
  return p0.map((value, index) =>
    (dh00 * value + dh10 * duration * v0[index] + dh01 * p1[index] +
      dh11 * duration * v1[index]) / duration);
}

function boundedHermite(p0, v0, p1, v1, alpha, duration, maxVisualVelocity) {
  const chord = p1.map((value, index) => value - p0[index]);
  const chordLength = Math.hypot(...chord);
  if (chordLength <= 1e-5 || duration <= 0) {
    return { position: p0.map((value, index) => value + chord[index] * alpha), mode: "linear-stop" };
  }
  const clampTangent = (velocity) => {
    const magnitude = Math.hypot(...velocity);
    const scale = magnitude > maxVisualVelocity ? maxVisualVelocity / magnitude : 1;
    return velocity.map((value) => value * scale);
  };
  const epsilon = 1e-5;
  const tangent0 = clampTangent(v0);
  const tangent1 = clampTangent(v1);
  const satisfiesBounds = (scale) => {
    const scaled0 = tangent0.map((value) => value * scale);
    const scaled1 = tangent1.map((value) => value * scale);
    for (const t of [0, 0.25, 0.5, 0.75, 1]) {
      const candidate = hermite(p0, scaled0, p1, scaled1, t, duration);
      const velocity = hermiteVelocity(p0, scaled0, p1, scaled1, t, duration);
      if (Math.hypot(...velocity) > maxVisualVelocity + epsilon) return false;
      const elapsed = duration * t;
      const remaining = duration * (1 - t);
      if (Math.hypot(...candidate.map((value, axis) => value - p0[axis])) >
            maxVisualVelocity * elapsed + epsilon ||
          Math.hypot(...candidate.map((value, axis) => value - p1[axis])) >
            maxVisualVelocity * remaining + epsilon) return false;
      if (chordLength > epsilon) {
        const progress = candidate.reduce(
          (sum, value, axis) => sum + (value - p0[axis]) * chord[axis], 0,
        ) / (chordLength * chordLength);
        const chordVelocity = velocity.reduce(
          (sum, value, axis) => sum + value * chord[axis], 0,
        );
        if (progress < -epsilon || progress > 1 + epsilon || chordVelocity < -epsilon) return false;
      }
    }
    return true;
  };
  let scale = 1;
  if (!satisfiesBounds(scale)) {
    if (!satisfiesBounds(0)) {
      return {
        position: p0.map((value, index) => value + chord[index] * alpha),
        mode: "linear-speed-bound",
      };
    }
    let lower = 0;
    let upper = 1;
    for (let iteration = 0; iteration < 16; iteration++) {
      const middle = (lower + upper) / 2;
      if (satisfiesBounds(middle)) lower = middle;
      else upper = middle;
    }
    scale = lower;
  }
  const candidate = hermite(
    p0,
    tangent0.map((value) => value * scale),
    p1,
    tangent1.map((value) => value * scale),
    alpha,
    duration,
  );
  return { position: candidate, mode: scale < 1 ? "hermite-tangent-limited" : "hermite" };
}

function boundedYawHermite(yaw0, yawRate0, yaw1, yawRate1, alpha, duration, maxYawRate) {
  const delta = angleDelta(yaw1, yaw0);
  if (Math.abs(delta) <= 1e-8 || duration <= 0) return normalizeDegrees(yaw0 + delta * alpha);
  const secant = delta / duration;
  const tangent = (rate) => {
    const bounded = Math.max(-maxYawRate, Math.min(maxYawRate, rate));
    return bounded * secant < 0 ? 0 : bounded;
  };
  let m0 = tangent(yawRate0);
  let m1 = tangent(yawRate1);
  const a = m0 / secant;
  const b = m1 / secant;
  const sum = a * a + b * b;
  if (sum > 9) {
    const scale = 3 / Math.sqrt(sum);
    m0 *= scale;
    m1 *= scale;
  }
  const unwrapped = hermite([yaw0], [m0], [yaw0 + delta], [m1], alpha, duration)[0];
  return normalizeDegrees(unwrapped);
}

export class RemoteSnapshotBuffer {
  constructor(maxSamples = 32, teleportThreshold = 10, {
    maxVisualVelocity = 16,
    maxVisualYawRate = 720,
  } = {}) {
    if (!Number.isInteger(maxSamples) || maxSamples < 2 || !(teleportThreshold > 0) ||
        !(maxVisualVelocity > 0) || !Number.isFinite(maxVisualVelocity) ||
        !(maxVisualYawRate > 0) || !Number.isFinite(maxVisualYawRate)) {
      throw new RangeError("remote interpolation limits must be positive");
    }
    this.maxSamples = maxSamples;
    this.teleportThreshold = teleportThreshold;
    this.maxVisualVelocity = maxVisualVelocity;
    this.maxVisualYawRate = maxVisualYawRate;
    this.samples = [];
    this.lastPushTeleported = false;
    this.lastSample = null;
  }

  push(snapshot) {
    if (!snapshot || !Number.isInteger(snapshot.tick) || snapshot.tick < 0 ||
        !Array.isArray(snapshot.position) || snapshot.position.length !== 3 ||
        !snapshot.position.every(Number.isFinite) || !Array.isArray(snapshot.velocity) ||
        snapshot.velocity.length !== 3 || !snapshot.velocity.every(Number.isFinite) ||
        !Number.isFinite(snapshot.character_yaw ?? 0) ||
        !Number.isFinite(snapshot.yaw_rate ?? 0)) {
      throw new TypeError("remote snapshot must contain finite tick, transform, and velocity data");
    }
    const newest = this.samples.at(-1);
    this.lastPushTeleported = false;
    if (
      newest &&
      snapshot.tick > newest.tick &&
      Math.hypot(...snapshot.position.map((value, index) => value - newest.position[index])) >
        this.teleportThreshold
    ) {
      this.samples = [];
      this.lastPushTeleported = true;
    }
    const previousIndex = this.samples.findIndex((sample) => sample.tick === snapshot.tick);
    if (previousIndex >= 0) this.samples.splice(previousIndex, 1);
    this.samples.push({
      ...snapshot,
      position: [...snapshot.position],
      velocity: [...snapshot.velocity],
    });
    this.samples.sort((a, b) => a.tick - b.tick);
    if (this.samples.length > this.maxSamples) {
      this.samples.splice(0, this.samples.length - this.maxSamples);
    }
  }

  sample(renderTick, tickRate, maxExtrapolationSeconds = 0.1) {
    if (this.samples.length === 0) return null;
    if (!Number.isFinite(renderTick) || !(tickRate > 0) || !Number.isFinite(tickRate) ||
        !(maxExtrapolationSeconds >= 0) || !Number.isFinite(maxExtrapolationSeconds)) {
      throw new RangeError("remote sampling rate and extrapolation limit are invalid");
    }
    if (renderTick <= this.samples[0].tick) {
      this.lastSample = { ...this.samples[0], extrapolation_seconds: 0, interpolation_mode: "buffer-edge" };
      return this.lastSample;
    }
    for (let index = 0; index < this.samples.length - 1; index++) {
      const a = this.samples[index];
      const b = this.samples[index + 1];
      if (renderTick < a.tick || renderTick > b.tick) continue;
      const ticks = b.tick - a.tick;
      const alpha = ticks > 0 ? (renderTick - a.tick) / ticks : 0;
      const duration = ticks / tickRate;
      const yaw = boundedYawHermite(
        Number(a.character_yaw ?? 0),
        Number(a.yaw_rate ?? 0),
        Number(b.character_yaw ?? 0),
        Number(b.yaw_rate ?? 0),
        alpha,
        duration,
        this.maxVisualYawRate,
      );
      const position = boundedHermite(
        a.position, a.velocity, b.position, b.velocity, alpha, duration, this.maxVisualVelocity,
      );
      this.lastSample = {
        ...b,
        position: position.position,
        character_yaw: yaw,
        extrapolation_seconds: 0,
        interpolation_mode: position.mode,
      };
      return this.lastSample;
    }
    const newest = this.samples.at(-1);
    const seconds = Math.min(
      maxExtrapolationSeconds,
      Math.max(0, (renderTick - newest.tick) / tickRate),
    );
    const velocityLength = Math.hypot(...newest.velocity);
    const velocityScale = velocityLength > this.maxVisualVelocity
      ? this.maxVisualVelocity / velocityLength
      : 1;
    this.lastSample = {
      ...newest,
      position: newest.position.map((value, index) =>
        value + newest.velocity[index] * velocityScale * seconds),
      character_yaw: normalizeDegrees(
        newest.character_yaw + Math.max(
          -this.maxVisualYawRate,
          Math.min(this.maxVisualYawRate, newest.yaw_rate ?? 0),
        ) * seconds,
      ),
      extrapolation_seconds: seconds,
      interpolation_mode: seconds > 0 ? "extrapolation" : "hold",
    };
    return this.lastSample;
  }
}

export class ServerClockEstimator {
  constructor(tickRate = 60) {
    this.setTickRate(tickRate);
    this.reset();
  }

  setTickRate(tickRate) {
    if (!(tickRate > 0)) throw new RangeError("server tick rate must be positive");
    this.tickRate = tickRate;
  }

  reset() {
    this.offsetTicks = null;
    this.lastObservedTick = null;
    this.lastArrivalMs = null;
    this.phaseErrorTicks = 0;
    this.observationCount = 0;
  }

  observe(serverTick, arrivalMs, rttMs = null) {
    if (!Number.isFinite(serverTick) || !Number.isFinite(arrivalMs)) {
      throw new TypeError("server clock sample must contain finite tick and arrival time");
    }
    if (this.lastObservedTick !== null && serverTick <= this.lastObservedTick) return false;
    const oneWayTicks = Number.isFinite(rttMs) && rttMs >= 0
      ? rttMs * this.tickRate / 2000
      : 0;
    const measuredOffset = serverTick + oneWayTicks - arrivalMs * this.tickRate / 1000;
    if (this.offsetTicks === null) {
      this.offsetTicks = measuredOffset;
    } else {
      const error = measuredOffset - this.offsetTicks;
      this.phaseErrorTicks = error;
      const boundedError = Math.max(-this.tickRate * 0.1, Math.min(this.tickRate * 0.1, error));
      this.offsetTicks += boundedError * 0.1;
    }
    this.lastObservedTick = serverTick;
    this.lastArrivalMs = arrivalMs;
    this.observationCount++;
    return true;
  }

  estimateTick(localTimeMs) {
    if (!Number.isFinite(localTimeMs)) throw new TypeError("local clock time must be finite");
    if (this.offsetTicks === null) return 0;
    return localTimeMs * this.tickRate / 1000 + this.offsetTicks;
  }
}

export class AdaptiveInterpolationDelay {
  constructor({ minMs = 50, maxMs = 300, initialMs = 100 } = {}) {
    if (!(minMs >= 0) || !(maxMs >= minMs) || initialMs < minMs || initialMs > maxMs) {
      throw new RangeError("interpolation delay bounds are invalid");
    }
    this.minMs = minMs;
    this.maxMs = maxMs;
    this.initialMs = initialMs;
    this.reset();
  }

  reset() {
    this.delayMs = this.initialMs;
    this.snapshotIntervalMs = 0;
    this.arrivalJitterMs = 0;
    this.rttMeanMs = null;
    this.rttVarianceMs = 0;
    this.lastTick = null;
    this.lastArrivalMs = null;
  }

  observe(serverTick, arrivalMs, tickRate, rttMs = null) {
    if (!(tickRate > 0) || !Number.isFinite(serverTick) || !Number.isFinite(arrivalMs)) {
      throw new TypeError("interpolation delay sample is invalid");
    }
    if (this.lastTick !== null && serverTick > this.lastTick) {
      const expectedInterval = (serverTick - this.lastTick) * 1000 / tickRate;
      const arrivalInterval = Math.max(0, arrivalMs - this.lastArrivalMs);
      this.snapshotIntervalMs = this.snapshotIntervalMs === 0
        ? expectedInterval
        : this.snapshotIntervalMs * 0.8 + expectedInterval * 0.2;
      const jitterSample = Math.abs(arrivalInterval - expectedInterval);
      this.arrivalJitterMs = this.arrivalJitterMs * 0.8 + jitterSample * 0.2;
    }
    if (Number.isFinite(rttMs) && rttMs >= 0) {
      if (this.rttMeanMs === null) {
        this.rttMeanMs = rttMs;
      } else {
        const error = Math.abs(rttMs - this.rttMeanMs);
        this.rttVarianceMs = this.rttVarianceMs * 0.8 + error * 0.2;
        this.rttMeanMs = this.rttMeanMs * 0.8 + rttMs * 0.2;
      }
    }
    this.lastTick = serverTick;
    this.lastArrivalMs = arrivalMs;
    const target = Math.max(
      this.snapshotIntervalMs * 1.25,
      30 + this.arrivalJitterMs * 2 + this.rttVarianceMs * 0.5,
    );
    const clamped = Math.max(this.minMs, Math.min(this.maxMs, target));
    const response = clamped > this.delayMs ? 0.25 : 0.08;
    this.delayMs += (clamped - this.delayMs) * response;
    return this.delayMs;
  }
}

export class NetworkSimulator {
  constructor() {
    this.latencyMs = 0;
    this.jitterMs = 0;
    this.packetLoss = 0;
    this.nextSendAt = 0;
  }

  configure({ latencyMs, jitterMs, packetLoss }) {
    this.latencyMs = latencyMs;
    this.jitterMs = jitterMs;
    this.packetLoss = packetLoss;
  }

  shouldDrop() {
    return Math.random() < this.packetLoss;
  }

  delayMs() {
    return Math.max(0, this.latencyMs + (Math.random() * 2 - 1) * this.jitterMs);
  }

  send(socket, message) {
    if (!socket || socket.readyState !== WebSocket.OPEN) return false;
    if (this.shouldDrop()) return true;
    const dueAt = Math.max(performance.now() + this.delayMs(), this.nextSendAt);
    this.nextSendAt = dueAt;
    window.setTimeout(() => {
      if (socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify(message));
    }, Math.max(0, dueAt - performance.now()));
    return true;
  }

  receive(callback, message) {
    if (this.shouldDrop()) return;
    window.setTimeout(callback, this.delayMs(), message);
  }
}
