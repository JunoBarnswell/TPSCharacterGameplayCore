export function inertialize(state, key, target, dt, halfLife = 0.12) {
  const current = state.get(key) ?? { value: structuredClone(target), velocity: {} };
  const decay = Math.exp(-Math.LN2 * Math.max(0, dt) / Math.max(1e-4, halfLife));
  const value = {};
  for (const [name, next] of Object.entries(target)) {
    const old = Number(current.value[name] ?? 0);
    const velocity = Number(current.velocity[name] ?? 0);
    const displacement = old - next;
    const c = velocity + displacement * (Math.LN2 / halfLife);
    value[name] = next + (displacement + c * dt) * decay;
    current.velocity[name] = (value[name] - old) / Math.max(dt, 1e-4);
  }
  current.value = value;
  state.set(key, current);
  return value;
}
