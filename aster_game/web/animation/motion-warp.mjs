import { angleDelta } from "../motion/movement-solver.mjs";

function vector(values, name) {
  if (!Array.isArray(values) || values.length !== 3 || !values.every(Number.isFinite)) {
    throw new TypeError(`${name} must be a finite 3D vector`);
  }
  return [...values];
}

function localToWorld(vector, yawDegrees) {
  const yaw = yawDegrees * Math.PI / 180;
  return [
    vector[0] * Math.cos(yaw) + vector[2] * Math.sin(yaw),
    vector[1],
    -vector[0] * Math.sin(yaw) + vector[2] * Math.cos(yaw),
  ];
}

function worldToLocal(vector, yawDegrees) {
  const yaw = yawDegrees * Math.PI / 180;
  return [
    vector[0] * Math.cos(yaw) - vector[2] * Math.sin(yaw),
    vector[1],
    vector[0] * Math.sin(yaw) + vector[2] * Math.cos(yaw),
  ];
}

export class MotionWarpTarget {
  constructor(name, position, yaw = 0) {
    if (typeof name !== "string" || name.length === 0 || !Number.isFinite(yaw)) {
      throw new TypeError("motion warp target requires a name and finite yaw");
    }
    this.name = name;
    this.position = Object.freeze(vector(position, "warp target position"));
    this.yaw = yaw;
    Object.freeze(this);
  }
}

export function warpRootMotionDelta(rootMotionDelta, currentTransform, remainingClipTranslation,
  target, weight = 1, remainingYawDegrees = 0) {
  if (!rootMotionDelta || !Array.isArray(rootMotionDelta.translation) ||
      !Number.isFinite(rootMotionDelta.yawDelta) || !currentTransform ||
      !Array.isArray(currentTransform.position) || !Number.isFinite(currentTransform.character_yaw) ||
      !(target instanceof MotionWarpTarget) || !Number.isFinite(weight) || weight < 0 || weight > 1 ||
      !Number.isFinite(remainingYawDegrees)) {
    throw new TypeError("motion warping requires root delta, current transform, target, and weight");
  }
  const step = vector(rootMotionDelta.translation, "root motion delta");
  const remaining = vector(remainingClipTranslation, "remaining clip translation");
  const current = vector(currentTransform.position, "current root position");
  const stepWorld = localToWorld(step, currentTransform.character_yaw);
  const remainingWorld = localToWorld(remaining, currentTransform.character_yaw);
  const baselineEnd = current.map((value, axis) => value + stepWorld[axis] + remainingWorld[axis]);
  const endpointError = target.position.map((value, axis) => value - baselineEnd[axis]);
  const stepLength = Math.hypot(...stepWorld);
  const remainingLength = Math.hypot(...remainingWorld);
  const translationShare = stepLength + remainingLength > 1e-8
    ? stepLength / (stepLength + remainingLength)
    : 1;
  const warpedWorldTranslation = stepWorld.map((value, axis) =>
    value + endpointError[axis] * translationShare * weight);
  const warpedTranslation = worldToLocal(warpedWorldTranslation, currentTransform.character_yaw);
  const baselineYawEnd = currentTransform.character_yaw + rootMotionDelta.yawDelta + remainingYawDegrees;
  const yawError = angleDelta(target.yaw, baselineYawEnd);
  const yawDenominator = Math.abs(rootMotionDelta.yawDelta) + Math.abs(remainingYawDegrees);
  const yawShare = yawDenominator > 1e-8 ? Math.abs(rootMotionDelta.yawDelta) / yawDenominator : 1;
  return {
    translation: warpedTranslation,
    yawDelta: rootMotionDelta.yawDelta + yawError * yawShare * weight,
    targetName: target.name,
    translationShare,
    yawShare,
  };
}
