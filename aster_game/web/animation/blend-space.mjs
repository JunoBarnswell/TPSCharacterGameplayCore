const samples = [
  ["idle", 0, 0],
  ["walk_forward", 0.28, 0], ["walk_backward", 0.28, 1],
  ["walk_left", 0.28, -0.5], ["walk_right", 0.28, 0.5],
  ["run_forward", 0.68, 0], ["run_backward", 0.68, 1],
  ["run_left", 0.68, -0.5], ["run_right", 0.68, 0.5],
  ["sprint", 1, 0],
];

export function evaluateBlendSpace(frame, maxSpeed) {
  const relativeDirection = ((frame.movementDirection + 180) % 360 - 180) / 180;
  const point = [Math.min(1, frame.horizontalSpeed / Math.max(0.01, maxSpeed)), relativeDirection];
  const raw = samples.map(([name, speed, direction]) => {
    const distance = Math.hypot(point[0] - speed, point[1] - direction);
    return [name, distance <= 1e-5 ? 1e10 : 1 / (distance * distance)];
  });
  const total = raw.reduce((sum, [, weight]) => sum + weight, 0);
  return Object.fromEntries(raw.map(([name, weight]) => [name, weight / total]));
}
