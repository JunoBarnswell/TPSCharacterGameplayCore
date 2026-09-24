import { createTransform, Pose, quaternionMultiply } from "./pose.mjs";

export function orientationWarpAngle(movementDirection, characterYaw, maxAngle = 90) {
  if (!Number.isFinite(movementDirection) || !Number.isFinite(characterYaw) ||
      !Number.isFinite(maxAngle) || maxAngle < 0) {
    throw new TypeError("orientation warp angles and limit must be finite and non-negative");
  }
  let difference = ((movementDirection - characterYaw + 540) % 360) - 180;
  difference = Math.max(-maxAngle, Math.min(maxAngle, difference));
  return difference;
}

export function warpPoseOrientation(pose, movementDirection, characterYaw, distribution, maxAngle = 90) {
  if (!(pose instanceof Pose) || !distribution || !Number.isFinite(movementDirection) ||
      !Number.isFinite(characterYaw) || !(maxAngle >= 0)) {
    throw new TypeError("pose orientation warp requires a pose, facing, and bone distribution");
  }
  const entries = Object.entries(distribution);
  const totalWeight = entries.reduce((sum, [, weight]) => {
    if (!Number.isFinite(weight) || weight < 0) throw new RangeError("warp bone weights must be non-negative");
    return sum + weight;
  }, 0);
  if (!(totalWeight > 0) || Math.abs(totalWeight - 1) > 1e-6) {
    throw new RangeError("orientation warp bone weights must sum to one");
  }
  const angle = orientationWarpAngle(movementDirection, characterYaw, maxAngle);
  const transforms = [...pose.localTransforms];
  const applied = {};
  for (const [name, weight] of entries) {
    const index = pose.skeleton.indexByName.get(name);
    if (index === undefined) throw new RangeError(`orientation warp bone '${name}' is missing`);
    if (weight === 0) continue;
    const transform = transforms[index];
    const halfAngle = angle * weight * Math.PI / 360;
    const yawRotation = [0, Math.sin(halfAngle), 0, Math.cos(halfAngle)];
    transforms[index] = createTransform(
      transform.translation,
      quaternionMultiply(transform.rotation, yawRotation),
      transform.scale,
    );
    applied[name] = angle * weight;
  }
  return { pose: new Pose(pose.skeleton, transforms), warpAngle: angle, appliedAngles: applied };
}
