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
  constructor(name, position, yaw = 0, window = null) {
    if (typeof name !== "string" || name.length === 0 || !Number.isFinite(yaw)) {
      throw new TypeError("motion warp target requires a name and finite yaw");
    }
    if (window !== null && !(window instanceof MotionWarpWindow)) {
      throw new TypeError("motion warp target window must be a MotionWarpWindow");
    }
    this.name = name;
    this.position = Object.freeze(vector(position, "warp target position"));
    this.yaw = yaw;
    this.window = window;
    Object.freeze(this);
  }
}

export class MotionWarpWindow {
  constructor(startSeconds, endSeconds, { blendInSeconds = 0, blendOutSeconds = 0 } = {}) {
    if (!Number.isFinite(startSeconds) || !Number.isFinite(endSeconds) ||
        !(endSeconds > startSeconds) || !Number.isFinite(blendInSeconds) ||
        !Number.isFinite(blendOutSeconds) || blendInSeconds < 0 || blendOutSeconds < 0 ||
        blendInSeconds + blendOutSeconds > endSeconds - startSeconds) {
      throw new RangeError("motion warp window bounds and blends are invalid");
    }
    this.startSeconds = startSeconds;
    this.endSeconds = endSeconds;
    this.blendInSeconds = blendInSeconds;
    this.blendOutSeconds = blendOutSeconds;
    Object.freeze(this);
  }

  weightAt(timeSeconds) {
    if (!Number.isFinite(timeSeconds)) {
      throw new TypeError("motion warp window time must be finite");
    }
    if (timeSeconds < this.startSeconds || timeSeconds > this.endSeconds) return 0;
    let weight = 1;
    if (this.blendInSeconds > 0) {
      const alpha = Math.max(0, Math.min(1,
        (timeSeconds - this.startSeconds) / this.blendInSeconds));
      weight = Math.min(weight, alpha * alpha * (3 - 2 * alpha));
    }
    if (this.blendOutSeconds > 0) {
      const alpha = Math.max(0, Math.min(1,
        (this.endSeconds - timeSeconds) / this.blendOutSeconds));
      weight = Math.min(weight, alpha * alpha * (3 - 2 * alpha));
    }
    return weight;
  }
}

export function warpRootMotionDelta(rootMotionDelta, currentTransform, remainingClipTranslation,
  target, weight = 1, remainingYawDegrees = 0, timeSeconds = null) {
  if (!rootMotionDelta || !Array.isArray(rootMotionDelta.translation) ||
      !Number.isFinite(rootMotionDelta.yawDelta) || !currentTransform ||
      !Array.isArray(currentTransform.position) || !Number.isFinite(currentTransform.character_yaw) ||
      !(target instanceof MotionWarpTarget) || !Number.isFinite(weight) || weight < 0 || weight > 1 ||
      !Number.isFinite(remainingYawDegrees) ||
      (timeSeconds !== null && !Number.isFinite(timeSeconds)) ||
      (target.window !== null && timeSeconds === null)) {
    throw new TypeError("motion warping requires root delta, current transform, target, and weight");
  }
  const windowWeight = target.window ? target.window.weightAt(timeSeconds) : 1;
  const effectiveWeight = weight * windowWeight;
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
    value + endpointError[axis] * translationShare * effectiveWeight);
  const warpedTranslation = worldToLocal(warpedWorldTranslation, currentTransform.character_yaw);
  const baselineYawEnd = currentTransform.character_yaw + rootMotionDelta.yawDelta + remainingYawDegrees;
  const yawError = angleDelta(target.yaw, baselineYawEnd);
  const yawDenominator = Math.abs(rootMotionDelta.yawDelta) + Math.abs(remainingYawDegrees);
  const yawShare = yawDenominator > 1e-8 ? Math.abs(rootMotionDelta.yawDelta) / yawDenominator : 1;
  return {
    translation: warpedTranslation,
    yawDelta: rootMotionDelta.yawDelta + yawError * yawShare * effectiveWeight,
    targetName: target.name,
    windowWeight,
    effectiveWeight,
    translationShare,
    yawShare,
  };
}
