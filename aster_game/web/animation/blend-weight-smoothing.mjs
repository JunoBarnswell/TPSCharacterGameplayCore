function normalizeWeights(weights) {
  const entries = Object.entries(weights).map(([name, weight]) => {
    if (!Number.isFinite(weight) || weight < 0) {
      throw new RangeError(`blend weight '${name}' must be finite and non-negative`);
    }
    return [name, weight];
  });
  const total = entries.reduce((sum, [, weight]) => sum + weight, 0);
  if (!(total > 0)) throw new RangeError("blend weights must have positive total weight");
  return Object.fromEntries(entries.map(([name, weight]) => [name, weight / total]));
}

export class BlendWeightSmoothing {
  constructor(halfLifeSeconds = 0.12) {
    if (!(halfLifeSeconds > 0)) throw new RangeError("blend weight half-life must be positive");
    this.halfLifeSeconds = halfLifeSeconds;
    this.values = null;
    this.velocities = {};
  }

  reset() {
    this.values = null;
    this.velocities = {};
  }

  update(targetWeights, dt) {
    if (!(dt >= 0) || !Number.isFinite(dt)) throw new RangeError("blend delta time must be non-negative");
    const target = normalizeWeights(targetWeights);
    if (this.values === null) {
      this.values = target;
      this.velocities = Object.fromEntries(Object.keys(target).map((name) => [name, 0]));
      return { ...this.values };
    }

    const names = new Set([...Object.keys(this.values), ...Object.keys(target)]);
    const decay = Math.exp(-Math.LN2 * dt / this.halfLifeSeconds);
    const smoothed = {};
    for (const name of names) {
      const old = this.values[name] ?? 0;
      const next = target[name] ?? 0;
      const velocity = this.velocities[name] ?? 0;
      const displacement = old - next;
      const coefficient = velocity + displacement * Math.LN2 / this.halfLifeSeconds;
      smoothed[name] = Math.max(0, next + (displacement + coefficient * dt) * decay);
    }
    const normalized = normalizeWeights(smoothed);
    this.velocities = Object.fromEntries([...names].map((name) => [
      name,
      dt > 0 ? ((normalized[name] ?? 0) - (this.values[name] ?? 0)) / dt : this.velocities[name] ?? 0,
    ]));
    this.values = normalized;
    return { ...normalized };
  }
}
