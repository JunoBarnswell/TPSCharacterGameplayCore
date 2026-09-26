import {
  createTransform,
  Pose,
  quaternionConjugate,
  quaternionFromTo,
  quaternionMultiply,
  quaternionSlerp,
  rotateVector,
  worldTransforms,
} from "./pose.mjs";

const validSides = new Set(["left", "right"]);

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function boneIndex(skeleton, name) {
  const index = skeleton.indexByName.get(name);
  if (index === undefined) throw new RangeError(`foot IK bone '${name}' is missing`);
  return index;
}

function worldDeltaToLocal(skeleton, worlds, bone, delta) {
  const parentIndex = skeleton.bones[bone].parentIndex;
  if (parentIndex < 0) return delta;
  const parent = worlds[parentIndex];
  const unrotated = rotateVector(quaternionConjugate(parent.rotation), delta);
  return unrotated.map((value, axis) => {
    if (Math.abs(parent.scale[axis]) < 1e-8) {
      throw new RangeError("foot IK cannot invert a zero-scaled parent transform");
    }
    return value / parent.scale[axis];
  });
}

function worldRotationToLocal(skeleton, worlds, bone, worldRotation) {
  const parentIndex = skeleton.bones[bone].parentIndex;
  return parentIndex < 0
    ? worldRotation
    : quaternionMultiply(quaternionConjugate(worlds[parentIndex].rotation), worldRotation);
}

export class FootLockState {
  constructor() {
    this.targets = new Map();
  }

  lock(side, position) {
    if (!validSides.has(side) || !Array.isArray(position) ||
        position.length !== 3 || !position.every(Number.isFinite)) {
      throw new TypeError("foot lock requires left/right side and a finite world position");
    }
    if (!this.targets.has(side)) this.targets.set(side, [...position]);
    return [...this.targets.get(side)];
  }

  release(side) {
    if (!validSides.has(side)) throw new TypeError("unknown foot lock side");
    this.targets.delete(side);
  }

  target(side) {
    const position = this.targets.get(side);
    return position ? [...position] : null;
  }
}

function clampVectorMagnitude(vector, maximum) {
  const length = Math.hypot(...vector);
  if (length <= maximum || length === 0) return vector;
  const scale = maximum / length;
  return vector.map((value) => value * scale);
}

function normalizeVector(vector) {
  const length = Math.hypot(...vector);
  if (!(length > 1e-8)) throw new RangeError("foot probe normal must be non-zero");
  return vector.map((value) => value / length);
}

export function solveTwoBoneLeg(pose, side, target, {
  kneePole = [0, 0, 1], maxExtension = 0.98, minimumBend = 0.015,
} = {}) {
  if (!(pose instanceof Pose) || !validSides.has(side) || !Array.isArray(target) ||
      target.length !== 3 || !target.every(Number.isFinite)) {
    throw new TypeError('two-bone leg requires a pose, side, and finite target');
  }
  const skeleton = pose.skeleton;
  const thigh = boneIndex(skeleton, `${side}_thigh`);
  const calf = boneIndex(skeleton, `${side}_calf`);
  const foot = boneIndex(skeleton, `${side}_foot`);
  if (skeleton.bones[calf].parentIndex !== thigh || skeleton.bones[foot].parentIndex !== calf) {
    throw new RangeError('two-bone leg must be a thigh → calf → foot chain');
  }
  const initial = worldTransforms(pose);
  const hip = initial[thigh].translation;
  const knee = initial[calf].translation;
  const ankle = initial[foot].translation;
  const upper = Math.hypot(...knee.map((v, i) => v - hip[i]));
  const lower = Math.hypot(...ankle.map((v, i) => v - knee[i]));
  const raw = target.map((v, i) => v - hip[i]);
  const rawDistance = Math.hypot(...raw);
  const direction = rawDistance > 1e-8 ? raw.map((v) => v / rawDistance) : [0, -1, 0];
  const distance = clamp(rawDistance, Math.abs(upper - lower) + minimumBend,
    (upper + lower) * maxExtension);
  const reachable = hip.map((v, i) => v + direction[i] * distance);
  const along = (upper * upper - lower * lower + distance * distance) / (2 * distance);
  const height = Math.sqrt(Math.max(0, upper * upper - along * along));
  const projected = kneePole.map((v, i) => v - direction[i] *
    kneePole.reduce((dot, component, axis) => dot + component * direction[axis], 0));
  const poleLength = Math.hypot(...projected);
  const pole = poleLength > 1e-8 ? projected.map((v) => v / poleLength) : [0, 0, 1];
  const desiredKnee = hip.map((v, i) => v + direction[i] * along + pole[i] * height);
  const thighRotation = quaternionMultiply(
    quaternionFromTo(knee.map((v, i) => v - hip[i]), desiredKnee.map((v, i) => v - hip[i])),
    initial[thigh].rotation);
  let transforms = [...pose.localTransforms];
  transforms[thigh] = createTransform(transforms[thigh].translation,
    worldRotationToLocal(skeleton, initial, thigh, thighRotation), transforms[thigh].scale);
  let solved = new Pose(skeleton, transforms);
  const rotated = worldTransforms(solved);
  const rotatedKnee = rotated[calf].translation;
  const rotatedAnkle = rotated[foot].translation;
  const calfRotation = quaternionMultiply(quaternionFromTo(
    rotatedAnkle.map((v, i) => v - rotatedKnee[i]),
    reachable.map((v, i) => v - rotatedKnee[i])), rotated[calf].rotation);
  transforms = [...solved.localTransforms];
  transforms[calf] = createTransform(transforms[calf].translation,
    worldRotationToLocal(skeleton, rotated, calf, calfRotation), transforms[calf].scale);
  solved = new Pose(skeleton, transforms);
  return { pose: solved, reachable: rawDistance <= (upper + lower) * maxExtension,
    error: Math.hypot(...worldTransforms(solved)[foot].translation.map((v, i) => v - target[i])) };
}

export function solveFootIK(pose, probes, {
  pelvisBone = "pelvis",
  leftFootBone = "left_foot",
  rightFootBone = "right_foot",
  maxFootOffset = 0.55,
  maxPelvisOffset = 0.3,
  previousPelvisOffset = null,
  dt = null,
  maxPelvisVelocity = 1.2,
  maxFootTilt = 35,
  maxFootAngularSpeed = 360,
  previousNormals = null,
  previousFootCorrections = null,
  maxFootCorrectionVelocity = 3,
} = {}) {
  if (!(pose instanceof Pose) || !probes || !Number.isFinite(maxFootOffset) || maxFootOffset < 0 ||
      !Number.isFinite(maxPelvisOffset) || maxPelvisOffset < 0) {
    throw new TypeError("foot IK requires a pose, two probes, and non-negative limits");
  }
  const skeleton = pose.skeleton;
  const pelvis = boneIndex(skeleton, pelvisBone);
  const feet = {
    left: boneIndex(skeleton, leftFootBone),
    right: boneIndex(skeleton, rightFootBone),
  };
  const initialWorld = worldTransforms(pose);
  const requiredOffsets = {};
  for (const side of ["left", "right"]) {
    const probe = probes[side];
    if (!probe || typeof probe.grounded !== "boolean") {
      throw new TypeError(`${side} foot probe must provide grounded state`);
    }
    const target = probe.lockedPosition ?? probe.position;
    if (probe.grounded && (!Array.isArray(target) || target.length !== 3 || !target.every(Number.isFinite))) {
      throw new TypeError(`${side} grounded foot probe must provide a finite position`);
    }
    requiredOffsets[side] = probe.grounded
      ? clamp(target[1] - initialWorld[feet[side]].translation[1], -maxFootOffset, maxFootOffset)
      : null;
  }

  const validOffsets = Object.values(requiredOffsets).filter((offset) => offset !== null);
  const desiredPelvisOffset = validOffsets.length
    ? clamp(Math.min(...validOffsets), -maxPelvisOffset, maxPelvisOffset)
    : 0;
  const pelvisOffset = Number.isFinite(previousPelvisOffset) && dt > 0
    ? previousPelvisOffset + clamp(desiredPelvisOffset - previousPelvisOffset,
      -maxPelvisVelocity * dt, maxPelvisVelocity * dt)
    : desiredPelvisOffset;
  const localTransforms = [...pose.localTransforms];
  const pelvisLocalDelta = worldDeltaToLocal(skeleton, initialWorld, pelvis, [0, pelvisOffset, 0]);
  const pelvisTransform = localTransforms[pelvis];
  localTransforms[pelvis] = createTransform(
    pelvisTransform.translation.map((value, axis) => value + pelvisLocalDelta[axis]),
    pelvisTransform.rotation,
    pelvisTransform.scale,
  );

  let adjustedPose = new Pose(skeleton, localTransforms);
  let adjustedWorld = worldTransforms(adjustedPose);
  const footOffsets = {};
  const footNormals = {};
  for (const side of ["left", "right"]) {
    const probe = probes[side];
    const foot = feet[side];
    if (!probe.grounded) {
      footOffsets[side] = 0;
      footNormals[side] = null;
      continue;
    }
    const targetPosition = probe.lockedPosition ?? probe.position;
    const footWorld = adjustedWorld[foot];
    let targetNormal = normalizeVector(probe.normal ?? [0, 1, 0]);
    const tilt = Math.acos(clamp(targetNormal[1], -1, 1));
    const tiltLimit = maxFootTilt * Math.PI / 180;
    if (tilt > tiltLimit) {
      const fraction = Math.sin(tiltLimit) / Math.max(1e-8, Math.sin(tilt));
      targetNormal = normalizeVector([targetNormal[0] * fraction,
        Math.cos(tiltLimit), targetNormal[2] * fraction]);
    }
    if (previousNormals?.[side] && dt > 0) {
      const previous = normalizeVector(previousNormals[side]);
      const angle = Math.acos(clamp(previous.reduce((sum, value, index) =>
        sum + value * targetNormal[index], 0), -1, 1));
      const maxAngle = maxFootAngularSpeed * Math.PI / 180 * dt;
      if (angle > maxAngle) {
        const blend = quaternionSlerp([0, 0, 0, 1], quaternionFromTo(previous, targetNormal),
          maxAngle / angle);
        targetNormal = normalizeVector(rotateVector(blend, previous));
      }
    }
    let residual = clampVectorMagnitude(
      targetPosition.map((value, axis) => value - footWorld.translation[axis]),
      maxFootOffset,
    );
    if (previousFootCorrections?.[side] && dt > 0) {
      const prior = previousFootCorrections[side];
      const change = clampVectorMagnitude(residual.map((value, axis) => value - prior[axis]),
        maxFootCorrectionVelocity * dt);
      residual = prior.map((value, axis) => value + change[axis]);
    }
    const hasLegChain = skeleton.indexByName.has(`${side}_thigh`) &&
      skeleton.indexByName.has(`${side}_calf`);
    if (hasLegChain) {
      adjustedPose = solveTwoBoneLeg(adjustedPose, side,
        footWorld.translation.map((value, axis) => value + residual[axis]),
        { kneePole: [side === 'left' ? -0.15 : 0.15, 0, 1] }).pose;
      adjustedWorld = worldTransforms(adjustedPose);
    }
    const localDelta = hasLegChain ? [0, 0, 0] : worldDeltaToLocal(skeleton, adjustedWorld, foot, residual);
    const localFoot = adjustedPose.localTransforms[foot];
    const solvedFootWorld = adjustedWorld[foot];
    const currentUp = rotateVector(solvedFootWorld.rotation, [0, 1, 0]);
    const targetRotation = quaternionMultiply(
      quaternionFromTo(currentUp, targetNormal),
      solvedFootWorld.rotation,
    );
    const localRotation = worldRotationToLocal(skeleton, adjustedWorld, foot, targetRotation);
    const nextTransforms = [...adjustedPose.localTransforms];
    nextTransforms[foot] = createTransform(
      localFoot.translation.map((value, axis) => value + localDelta[axis]),
      localRotation,
      localFoot.scale,
    );
    adjustedPose = new Pose(skeleton, nextTransforms);
    adjustedWorld = worldTransforms(adjustedPose);
    footOffsets[side] = residual;
    footNormals[side] = targetNormal;
  }
  return {
    pose: adjustedPose,
    pelvisOffset,
    footOffsets,
    footNormals,
  };
}
