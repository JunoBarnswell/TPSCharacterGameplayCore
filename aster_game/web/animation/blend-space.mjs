const samples = [
  ["idle", 0, 0],
  ["walk_forward", 0.28, 0], ["walk_backward", 0.28, 1],
  ["walk_left", 0.28, -0.5], ["walk_right", 0.28, 0.5],
  ["run_forward", 0.68, 0], ["run_backward", 0.68, 1],
  ["run_left", 0.68, -0.5], ["run_right", 0.68, 0.5],
  ["sprint", 1, 0],
];

function circularDistance(left, right) {
  const distance = Math.abs(left - right);
  return Math.min(distance, 2 - distance);
}

function normalizeWeights(entries) {
  const total = entries.reduce((sum, [, weight]) => sum + Math.max(0, weight), 0);
  if (!(total > 0)) throw new RangeError("blend weights must have positive total weight");
  return Object.fromEntries(entries.map(([name, weight]) => [name, Math.max(0, weight) / total]));
}

export function evaluateBlendSpace(frame, maxSpeed, neighborCount = 3) {
  if (!(maxSpeed > 0) || !Number.isInteger(neighborCount) || neighborCount < 1) {
    throw new RangeError("blend-space speed and neighbor count must be positive");
  }
  const direction = ((frame.movementDirection / 180 + 1) % 2 + 2) % 2 - 1;
  const speed = Math.max(0, Math.min(1, frame.horizontalSpeed / maxSpeed));
  const local = samples.map(([name, sampleSpeed, sampleDirection]) => ({
    name,
    distance: Math.hypot(speed - sampleSpeed, circularDistance(direction, sampleDirection)),
  })).sort((a, b) => a.distance - b.distance || a.name.localeCompare(b.name))
    .slice(0, Math.min(neighborCount, samples.length));
  const exact = local.find(({ distance }) => distance <= 1e-6);
  const weights = exact
    ? [[exact.name, 1]]
    : local.map(({ name, distance }) => [name, 1 / (distance * distance)]);
  const normalized = normalizeWeights(weights);
  return Object.fromEntries(samples.map(([name]) => [name, normalized[name] ?? 0]));
}
