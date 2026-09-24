import { Pose, worldTransforms } from "../pose.mjs";

function directionFromQuaternion([x, y, z, w]) {
  const yaw = Math.atan2(2 * (w * y + x * z), 1 - 2 * (y * y + z * z));
  return [Math.sin(yaw), Math.cos(yaw)];
}

function requiredBoneIndex(skeleton, name) {
  const index = skeleton.indexByName.get(name);
  if (index === undefined) throw new RangeError(`pose feature bone '${name}' is missing`);
  return index;
}

function featureVector(values, length, name) {
  if (!Array.isArray(values) || values.length !== length || !values.every(Number.isFinite)) {
    throw new TypeError(`${name} must be a finite ${length}D vector`);
  }
  return [...values];
}

export function flattenPoseFeatureVector(features) {
  if (!features || typeof features !== "object" || !Array.isArray(features.trajectory)) {
    throw new TypeError("pose features must contain vectors and a trajectory");
  }
  const parts = [
    featureVector(features.rootVelocity, 3, "root velocity"),
    featureVector(features.facing, 2, "facing"),
    featureVector(features.pelvisPosition, 3, "pelvis position"),
    featureVector(features.leftFootPosition, 3, "left foot position"),
    featureVector(features.rightFootPosition, 3, "right foot position"),
    featureVector(features.pelvisVelocity, 3, "pelvis velocity"),
    featureVector(features.leftFootVelocity, 3, "left foot velocity"),
    featureVector(features.rightFootVelocity, 3, "right foot velocity"),
  ];
  for (const sample of features.trajectory) {
    if (!sample || typeof sample !== "object") {
      throw new TypeError("pose trajectory features must be structured samples");
    }
    parts.push(
      featureVector(sample.position, 3, "trajectory position"),
      featureVector(sample.facing, 2, "trajectory facing"),
      featureVector(sample.velocity, 3, "trajectory velocity"),
    );
  }
  return parts.flat();
}

function relativePosition(world, index, rootPosition) {
  return world[index].translation.map((value, axis) => value - rootPosition[axis]);
}

function positionVelocity(currentPosition, previousPosition, dt) {
  return currentPosition.map((value, axis) => (value - previousPosition[axis]) / dt);
}

export function extractPoseFeatures({
  pose,
  rootVelocity,
  trajectory = [],
  previousSample = null,
  dt = 1 / 60,
  bones = { root: "root", pelvis: "pelvis", leftFoot: "left_foot", rightFoot: "right_foot" },
}) {
  if (!(pose instanceof Pose) || !(dt > 0) || !Number.isFinite(dt) || !Array.isArray(trajectory)) {
    throw new TypeError("pose feature extraction requires a pose, trajectory, and positive timestep");
  }
  const rootVelocityVector = featureVector(rootVelocity, 3, "root velocity");
  const world = worldTransforms(pose);
  const indices = Object.fromEntries(Object.entries(bones).map(([key, name]) => [
    key,
    requiredBoneIndex(pose.skeleton, name),
  ]));
  const rootPosition = world[indices.root].translation;
  const pelvisPosition = relativePosition(world, indices.pelvis, rootPosition);
  const leftFootPosition = relativePosition(world, indices.leftFoot, rootPosition);
  const rightFootPosition = relativePosition(world, indices.rightFoot, rootPosition);
  const previousPositions = previousSample?.bonePositions ?? null;
  const leftFootVelocity = previousPositions
    ? positionVelocity(world[indices.leftFoot].translation, previousPositions[bones.leftFoot], dt)
    : [0, 0, 0];
  const rightFootVelocity = previousPositions
    ? positionVelocity(world[indices.rightFoot].translation, previousPositions[bones.rightFoot], dt)
    : [0, 0, 0];
  const pelvisVelocity = previousPositions
    ? positionVelocity(world[indices.pelvis].translation, previousPositions[bones.pelvis], dt)
    : [0, 0, 0];
  const trajectoryFeatures = trajectory.map((sample) => {
    if (!sample || !Array.isArray(sample.position) || sample.position.length !== 3 ||
        !sample.position.every(Number.isFinite) || !Array.isArray(sample.velocity) ||
        sample.velocity.length !== 3 || !sample.velocity.every(Number.isFinite) ||
        !Number.isFinite(sample.facing)) {
      throw new TypeError("trajectory feature samples must contain finite position, velocity, and facing");
    }
    const facingRadians = sample.facing * Math.PI / 180;
    return {
      position: sample.position.map((value, axis) => value - rootPosition[axis]),
      facing: [Math.sin(facingRadians), Math.cos(facingRadians)],
      velocity: [...sample.velocity],
    };
  });
  const facing = directionFromQuaternion(world[indices.root].rotation);
  const features = {
    rootVelocity: rootVelocityVector,
    facing,
    pelvisPosition,
    pelvisVelocity,
    leftFootPosition,
    rightFootPosition,
    leftFootVelocity,
    rightFootVelocity,
    trajectory: trajectoryFeatures,
  };
  return { ...features, vector: flattenPoseFeatureVector(features) };
}
