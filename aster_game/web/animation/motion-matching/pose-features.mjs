import { Pose, worldTransforms } from "../pose.mjs";

function directionFromQuaternion([x, y, z, w]) {
  const yaw = Math.atan2(2 * (w * y + x * z), 1 - 2 * (y * y + z * z));
  return [Math.sin(yaw), Math.cos(yaw)];
}

function localYawVector(vector, yaw) {
  const radians = yaw * Math.PI / 180;
  const sine = Math.sin(radians);
  const cosine = Math.cos(radians);
  return [vector[0] * cosine - vector[2] * sine, vector[1],
    vector[0] * sine + vector[2] * cosine];
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
    featureVector(features.contacts, 2, "foot contacts"),
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

export function groupPoseFeatures(features) {
  const flatten = (vectors) => vectors.flatMap((vector) => [...vector]);
  return {
    pose: flatten([features.pelvisPosition, features.leftFootPosition, features.rightFootPosition]),
    trajectory: flatten(features.trajectory.map(({ position }) => position)),
    velocity: flatten([features.rootVelocity, features.pelvisVelocity,
      features.leftFootVelocity, features.rightFootVelocity,
      ...features.trajectory.map(({ velocity }) => velocity)]),
    facing: flatten([features.facing, ...features.trajectory.map(({ facing }) => facing)]),
    contacts: [...features.contacts],
  };
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
  contacts = { left: false, right: false },
  dt = 1 / 60,
  bones = { root: "root", pelvis: "pelvis", leftFoot: "left_foot", rightFoot: "right_foot" },
}) {
  if (!(pose instanceof Pose) || !(dt > 0) || !Number.isFinite(dt) || !Array.isArray(trajectory) ||
      !contacts || typeof contacts !== "object") {
    throw new TypeError("pose feature extraction requires a pose, trajectory, and positive timestep");
  }
  const rootVelocityVector = featureVector(rootVelocity, 3, "root velocity");
  const world = worldTransforms(pose);
  const indices = Object.fromEntries(Object.entries(bones).map(([key, name]) => [
    key,
    requiredBoneIndex(pose.skeleton, name),
  ]));
  const rootPosition = world[indices.root].translation;
  const [rootSin, rootCos] = directionFromQuaternion(world[indices.root].rotation);
  const rootYaw = Math.atan2(rootSin, rootCos) * 180 / Math.PI;
  const canonical = (vector) => localYawVector(vector, rootYaw);
  const pelvisPosition = canonical(relativePosition(world, indices.pelvis, rootPosition));
  const leftFootPosition = canonical(relativePosition(world, indices.leftFoot, rootPosition));
  const rightFootPosition = canonical(relativePosition(world, indices.rightFoot, rootPosition));
  const previousPositions = previousSample?.bonePositions ?? null;
  const leftFootVelocity = previousPositions
    ? canonical(positionVelocity(world[indices.leftFoot].translation, previousPositions[bones.leftFoot], dt))
    : [0, 0, 0];
  const rightFootVelocity = previousPositions
    ? canonical(positionVelocity(world[indices.rightFoot].translation, previousPositions[bones.rightFoot], dt))
    : [0, 0, 0];
  const pelvisVelocity = previousPositions
    ? canonical(positionVelocity(world[indices.pelvis].translation, previousPositions[bones.pelvis], dt))
    : [0, 0, 0];
  const trajectoryFeatures = trajectory.map((sample) => {
    if (!sample || !Array.isArray(sample.position) || sample.position.length !== 3 ||
        !sample.position.every(Number.isFinite) || !Array.isArray(sample.velocity) ||
        sample.velocity.length !== 3 || !sample.velocity.every(Number.isFinite) ||
        !Number.isFinite(sample.facing)) {
      throw new TypeError("trajectory feature samples must contain finite position, velocity, and facing");
    }
    const facingRadians = (sample.facing - rootYaw) * Math.PI / 180;
    return {
      position: canonical(sample.position.map((value, axis) => value - rootPosition[axis])),
      facing: [Math.sin(facingRadians), Math.cos(facingRadians)],
      velocity: canonical(sample.velocity),
    };
  });
  const facing = [0, 1];
  const contactWeight = (value, name) => {
    if (typeof value === "boolean") return Number(value);
    if (Number.isFinite(value) && value >= 0 && value <= 1) return value;
    throw new TypeError(`${name} contact must be boolean or a weight in [0, 1]`);
  };
  const features = {
    rootVelocity: canonical(rootVelocityVector),
    facing,
    contacts: [
      contactWeight(contacts.left, "left foot"),
      contactWeight(contacts.right, "right foot"),
    ],
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
