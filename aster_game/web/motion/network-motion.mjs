import { predictMovementStep } from "./movement-solver.mjs";

export class PredictionHistory {
  constructor(limit = 256) {
    this.limit = limit;
    this.state = null;
    this.inputs = [];
  }

  reset(authoritative) {
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    this.inputs = [];
  }

  predict(input, tuning, dt) {
    if (!this.state) return null;
    this.state = predictMovementStep(this.state, input, tuning, dt);
    this.inputs.push({ input: { ...input }, state: structuredClone(this.state), sentAt: input.sentAt });
    if (this.inputs.length > this.limit) this.inputs.shift();
    return this.state;
  }

  reconcile(authoritative, ack, tuning, dt) {
    if (!this.state || !authoritative) {
      this.reset(authoritative);
      return this.state;
    }
    this.inputs = this.inputs.filter(({ input }) => input.sequence > ack);
    this.state = {
      ...authoritative,
      position: [...authoritative.position],
      velocity: [...authoritative.velocity],
    };
    for (const entry of this.inputs) {
      this.state = predictMovementStep(this.state, entry.input, tuning, dt);
      entry.state = structuredClone(this.state);
    }
    return this.state;
  }
}

export class VisualSmoothing {
  constructor(duration = 0.12, snapDistance = 3.0) {
    this.duration = duration;
    this.snapDistance = snapDistance;
    this.offset = [0, 0, 0];
  }

  correct(previousVisualPosition, simulationPosition) {
    const error = previousVisualPosition.map((value, index) => value - simulationPosition[index]);
    if (Math.hypot(...error) > this.snapDistance) this.offset = [0, 0, 0];
    else this.offset = error;
  }

  render(simulationPosition, dt) {
    const decay = Math.exp(-Math.max(0, dt) / Math.max(1e-4, this.duration));
    this.offset = this.offset.map((value) => value * decay);
    return simulationPosition.map((value, index) => value + this.offset[index]);
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

export class RemoteSnapshotBuffer {
  constructor(maxSamples = 32, teleportThreshold = 10) {
    this.maxSamples = maxSamples;
    this.teleportThreshold = teleportThreshold;
    this.samples = [];
  }

  push(snapshot) {
    const newest = this.samples.at(-1);
    if (
      newest &&
      Math.hypot(...snapshot.position.map((value, index) => value - newest.position[index])) >
        this.teleportThreshold
    ) {
      this.samples = [];
    }
    const previousIndex = this.samples.findIndex((sample) => sample.tick === snapshot.tick);
    if (previousIndex >= 0) this.samples.splice(previousIndex, 1);
    this.samples.push(snapshot);
    this.samples.sort((a, b) => a.tick - b.tick);
    if (this.samples.length > this.maxSamples) {
      this.samples.splice(0, this.samples.length - this.maxSamples);
    }
  }

  sample(renderTick, tickRate, maxExtrapolationSeconds = 0.1) {
    if (this.samples.length === 0) return null;
    if (renderTick <= this.samples[0].tick) return this.samples[0];
    for (let index = 0; index < this.samples.length - 1; index++) {
      const a = this.samples[index];
      const b = this.samples[index + 1];
      if (renderTick < a.tick || renderTick > b.tick) continue;
      const ticks = b.tick - a.tick;
      const alpha = ticks > 0 ? (renderTick - a.tick) / ticks : 0;
      const duration = ticks / tickRate;
      const yawDelta = ((b.character_yaw - a.character_yaw + 540) % 360) - 180;
      return {
        ...b,
        position: hermite(a.position, a.velocity, b.position, b.velocity, alpha, duration),
        character_yaw: a.character_yaw + yawDelta * alpha,
      };
    }
    const newest = this.samples.at(-1);
    const seconds = Math.min(
      maxExtrapolationSeconds,
      Math.max(0, (renderTick - newest.tick) / tickRate),
    );
    return {
      ...newest,
      position: newest.position.map((value, index) => value + newest.velocity[index] * seconds),
    };
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
