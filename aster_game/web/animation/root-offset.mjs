import { createTransform, Pose, worldTransforms } from "./pose.mjs";

export function applyVisualRootOffset(pose, simulationPosition, {
  rootBone = "root",
  maxOffset = 0.75,
} = {}) {
  if (!(pose instanceof Pose) || !Array.isArray(simulationPosition) ||
      simulationPosition.length !== 3 || !simulationPosition.every(Number.isFinite) ||
      !Number.isFinite(maxOffset) || maxOffset < 0) {
    throw new TypeError("visual root offset requires a pose, capsule position, and non-negative limit");
  }
  const rootIndex = pose.skeleton.indexByName.get(rootBone);
  if (rootIndex === undefined) throw new RangeError(`root offset bone '${rootBone}' is missing`);
  if (pose.skeleton.bones[rootIndex].parentIndex >= 0) {
    throw new RangeError("visual root offset bone must be a skeleton root");
  }
  const rootWorld = worldTransforms(pose)[rootIndex];
  const rawOffset = simulationPosition.map((value, axis) => value - rootWorld.translation[axis]);
  const distance = Math.hypot(...rawOffset);
  const scale = distance > maxOffset && distance > 0 ? maxOffset / distance : 1;
  const offset = rawOffset.map((value) => value * scale);
  const root = pose.localTransforms[rootIndex];
  const transforms = [...pose.localTransforms];
  transforms[rootIndex] = createTransform(
    root.translation.map((value, axis) => value + offset[axis]),
    root.rotation,
    root.scale,
  );
  return { pose: new Pose(pose.skeleton, transforms), offset, clamped: scale < 1 };
}
