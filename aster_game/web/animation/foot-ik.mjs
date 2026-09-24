import {
  createTransform,
  Pose,
  quaternionConjugate,
  quaternionFromTo,
  quaternionMultiply,
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

export function solveFootIK(pose, probes, {
  pelvisBone = "pelvis",
  leftFootBone = "left_foot",
  rightFootBone = "right_foot",
  maxFootOffset = 0.55,
  maxPelvisOffset = 0.3,
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
  const pelvisOffset = validOffsets.length
    ? clamp(Math.min(...validOffsets), -maxPelvisOffset, maxPelvisOffset)
    : 0;
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
    const targetNormal = normalizeVector(probe.normal ?? [0, 1, 0]);
    const residual = clampVectorMagnitude(
      targetPosition.map((value, axis) => value - footWorld.translation[axis]),
      maxFootOffset,
    );
    const localDelta = worldDeltaToLocal(skeleton, adjustedWorld, foot, residual);
    const localFoot = adjustedPose.localTransforms[foot];
    const currentUp = rotateVector(footWorld.rotation, [0, 1, 0]);
    const targetRotation = quaternionMultiply(
      quaternionFromTo(currentUp, targetNormal),
      footWorld.rotation,
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
