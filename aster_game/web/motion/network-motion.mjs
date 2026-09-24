import { angleDelta, predictMovementStep } from "./movement-solver.mjs";

export class PredictionHistory {
  constructor(limit = 256, largeCorrectionThreshold = 0.5) {
    this.limit = limit;
    this.largeCorrectionThreshold = largeCorrectionThreshold;
    this.state = null;
    this.inputs = [];
    this.metrics = {
      position_error: 0,
      rotation_error: 0,
      velocity_error: 0,
      reconciliation_count: 0,
      large_correction_count: 0,
    };
  }

  reset(authoritative) {
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    this.inputs = [];
    this.metrics = {
      position_error: 0,
      rotation_error: 0,
      velocity_error: 0,
      reconciliation_count: 0,
      large_correction_count: 0,
    };
  }

  predict(input, tuning, dt, collisionWorld = null) {
    if (!this.state) return null;
    this.state = predictMovementStep(this.state, input, tuning, dt, collisionWorld);
    this.inputs.push({ input: { ...input }, state: structuredClone(this.state), sentAt: input.sentAt });
    if (this.inputs.length > this.limit) this.inputs.shift();
    return this.state;
  }

  reconcile(authoritative, ack, tuning, dt, collisionWorld = null) {
    if (!this.state || !authoritative) {
      this.reset(authoritative);
      return this.state;
    }
    const positionDifference = this.state.position.map(
      (value, axis) => value - authoritative.position[axis],
    );
    const velocityDifference = this.state.velocity.map(
      (value, axis) => value - authoritative.velocity[axis],
    );
    const positionError = Math.hypot(...positionDifference);
    this.metrics.position_error = positionError;
    this.metrics.rotation_error = Math.abs(
      angleDelta(this.state.character_yaw, authoritative.character_yaw),
    );
    this.metrics.velocity_error = Math.hypot(...velocityDifference);
    this.metrics.reconciliation_count++;
    if (positionError > this.largeCorrectionThreshold) this.metrics.large_correction_count++;
    this.inputs = this.inputs.filter(({ input }) => input.sequence > ack);
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    for (const entry of this.inputs) {
      this.state = predictMovementStep(this.state, entry.input, tuning, dt, collisionWorld);
      entry.state = structuredClone(this.state);
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

function boundedHermite(p0, v0, p1, v1, alpha, duration, maxVisualVelocity) {
  const chord = p1.map((value, index) => value - p0[index]);
  const chordLength = Math.hypot(...chord);
  if (chordLength <= 1e-5 || duration <= 0) {
    return { position: p0.map((value, index) => value + chord[index] * alpha), mode: "linear-stop" };
  }
  const direction = chord.map((value) => value / chordLength);
  const maxTangent = Math.min(maxVisualVelocity, 3 * chordLength / duration);
  const clampTangent = (velocity) => {
    const along = velocity.reduce((sum, value, index) => sum + value * direction[index], 0);
    const magnitude = Math.max(0, Math.min(maxTangent, along));
    return direction.map((value) => value * magnitude);
  };
  const candidate = hermite(p0, clampTangent(v0), p1, clampTangent(v1), alpha, duration);
  const epsilon = 1e-5;
  const outsideSegment = candidate.some((value, index) =>
    value < Math.min(p0[index], p1[index]) - epsilon ||
    value > Math.max(p0[index], p1[index]) + epsilon);
  if (outsideSegment) {
    return { position: p0.map((value, index) => value + chord[index] * alpha), mode: "linear-overshoot" };
  }
  const elapsed = duration * alpha;
  const remaining = duration * (1 - alpha);
  const fromStart = Math.hypot(...candidate.map((value, index) => value - p0[index]));
  const toEnd = Math.hypot(...candidate.map((value, index) => value - p1[index]));
  if (fromStart > maxVisualVelocity * elapsed + epsilon ||
      toEnd > maxVisualVelocity * remaining + epsilon) {
    return { position: p0.map((value, index) => value + chord[index] * alpha), mode: "linear-speed-bound" };
  }
  return { position: candidate, mode: "hermite" };
}

export class RemoteSnapshotBuffer {
  constructor(maxSamples = 32, teleportThreshold = 10, { maxVisualVelocity = 16 } = {}) {
    if (!Number.isInteger(maxSamples) || maxSamples < 2 || !(teleportThreshold > 0) ||
        !(maxVisualVelocity > 0)) {
      throw new RangeError("remote interpolation limits must be positive");
    }
    this.maxSamples = maxSamples;
    this.teleportThreshold = teleportThreshold;
    this.maxVisualVelocity = maxVisualVelocity;
    this.samples = [];
    this.lastPushTeleported = false;
    this.lastSample = null;
  }

  push(snapshot) {
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
    if (!(tickRate > 0) || !(maxExtrapolationSeconds >= 0)) {
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
      const yawDelta = ((b.character_yaw - a.character_yaw + 540) % 360) - 180;
      const position = boundedHermite(
        a.position, a.velocity, b.position, b.velocity, alpha, duration, this.maxVisualVelocity,
      );
      this.lastSample = {
        ...b,
        position: position.position,
        character_yaw: normalizeDegrees(a.character_yaw + yawDelta * alpha),
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
        newest.character_yaw + Math.max(-720, Math.min(720, newest.yaw_rate ?? 0)) * seconds,
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
