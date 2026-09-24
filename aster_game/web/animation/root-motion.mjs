import {
  Pose,
  quaternionConjugate,
  quaternionMultiply,
  rotateVector,
  worldTransforms,
} from "./pose.mjs";
import { angleDelta } from "../motion/movement-solver.mjs";

function rootIndex(pose, boneName) {
  if (!(pose instanceof Pose)) throw new TypeError("root motion requires a pose");
  const index = pose.skeleton.indexByName.get(boneName);
  if (index === undefined) throw new RangeError(`root motion bone '${boneName}' is missing`);
  return index;
}

function yawFromQuaternion([x, y, z, w]) {
  return Math.atan2(2 * (w * y + x * z), 1 - 2 * (y * y + z * z)) * 180 / Math.PI;
}

export function extractRootMotionDelta(previousPose, currentPose, boneName = "root") {
  if (!(previousPose instanceof Pose) || !(currentPose instanceof Pose) ||
      previousPose.skeleton !== currentPose.skeleton) {
    throw new TypeError("root motion poses must share one skeleton");
  }
  const index = rootIndex(currentPose, boneName);
  const previous = worldTransforms(previousPose)[index];
  const current = worldTransforms(currentPose)[index];
  const rotationDelta = quaternionMultiply(current.rotation, quaternionConjugate(previous.rotation));
  const worldTranslationDelta = current.translation.map((value, axis) => value - previous.translation[axis]);
  return {
    translation: rotateVector(quaternionConjugate(previous.rotation), worldTranslationDelta),
    rotation: rotationDelta,
    yawDelta: angleDelta(yawFromQuaternion(current.rotation), yawFromQuaternion(previous.rotation)),
  };
}

export function applyRootMotionDelta(simulationTransform, delta, weight = 1) {
  if (!simulationTransform || !Array.isArray(simulationTransform.position) ||
      simulationTransform.position.length !== 3 || !simulationTransform.position.every(Number.isFinite) ||
      !delta || !Array.isArray(delta.translation) || delta.translation.length !== 3 ||
      !delta.translation.every(Number.isFinite) || !Number.isFinite(delta.yawDelta) ||
      !Number.isFinite(weight) || weight < 0 || weight > 1) {
    throw new TypeError("root motion application requires finite transforms and a weight in [0, 1]");
  }
  const yaw = Number(simulationTransform.character_yaw ?? 0);
  if (!Number.isFinite(yaw)) throw new TypeError("root motion character yaw must be finite");
  const radians = yaw * Math.PI / 180;
  const local = delta.translation.map((value) => value * weight);
  const worldDelta = [
    local[0] * Math.cos(radians) + local[2] * Math.sin(radians),
    local[1],
    -local[0] * Math.sin(radians) + local[2] * Math.cos(radians),
  ];
  return {
    position: simulationTransform.position.map((value, axis) => value + worldDelta[axis]),
    character_yaw: angleDelta(yaw + delta.yawDelta * weight, 0),
  };
}
