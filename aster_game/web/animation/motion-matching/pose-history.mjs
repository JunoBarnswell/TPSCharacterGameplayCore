import { Pose, worldTransforms } from "../pose.mjs";
import { extractPoseFeatures } from "./pose-features.mjs";

const defaultBones = Object.freeze(["root", "pelvis", "left_foot", "right_foot"]);
const featureBones = Object.freeze(["root", "pelvis", "left_foot", "right_foot"]);
const featureVectorNames = Object.freeze([
  "rootVelocity", "facing", "pelvisPosition", "pelvisVelocity",
  "leftFootPosition", "rightFootPosition", "leftFootVelocity", "rightFootVelocity",
]);

function immutableFeatures(features) {
  const result = {};
  for (const name of featureVectorNames) result[name] = Object.freeze([...features[name]]);
  result.trajectory = Object.freeze(features.trajectory.map((sample) => Object.freeze({
    position: Object.freeze([...sample.position]),
    facing: Object.freeze([...sample.facing]),
    velocity: Object.freeze([...sample.velocity]),
  })));
  result.vector = Object.freeze([...features.vector]);
  return Object.freeze(result);
}

export class PoseHistory {
  constructor(capacity = 180, boneNames = defaultBones) {
    if (!Number.isInteger(capacity) || capacity < 1 || !Array.isArray(boneNames) ||
        boneNames.length < featureBones.length || new Set(boneNames).size !== boneNames.length ||
        featureBones.some((name) => !boneNames.includes(name))) {
      throw new RangeError("pose history requires positive capacity, unique bones, and root/pelvis/left_foot/right_foot");
    }
    this.capacity = capacity;
    this.boneNames = Object.freeze([...boneNames]);
    this.skeleton = null;
    this.samples = [];
  }

  push(pose, {
    tick,
    rootVelocity,
    trajectory = [],
    clipName = "unassigned",
    timeSeconds = 0,
    dt = 1 / 60,
  }) {
    if (!(pose instanceof Pose) || !Number.isInteger(tick) ||
        !Number.isFinite(timeSeconds) || timeSeconds < 0 ||
        typeof clipName !== "string" || clipName.length === 0 ||
        !(dt > 0) || !Number.isFinite(dt)) {
      throw new TypeError("pose history sample metadata is invalid");
    }
    if (this.skeleton !== null && this.skeleton !== pose.skeleton) {
      throw new TypeError("pose history cannot mix different skeleton instances");
    }
    const previousSample = this.samples.at(-1) ?? null;
    if (previousSample && tick <= previousSample.tick) {
      throw new RangeError("pose history ticks must increase monotonically");
    }
    const world = worldTransforms(pose);
    const bonePositions = {};
    const boneRotations = {};
    for (const name of this.boneNames) {
      const index = pose.skeleton.indexByName.get(name);
      if (index === undefined) throw new RangeError(`pose history bone '${name}' is missing`);
      bonePositions[name] = Object.freeze([...world[index].translation]);
      boneRotations[name] = Object.freeze([...world[index].rotation]);
    }
    const features = extractPoseFeatures({
      pose,
      rootVelocity,
      trajectory,
      previousSample,
      dt,
      bones: {
        root: "root",
        pelvis: "pelvis",
        leftFoot: "left_foot",
        rightFoot: "right_foot",
      },
    });
    const sample = Object.freeze({
      id: `${clipName}@${timeSeconds.toFixed(6)}`,
      tick,
      pose,
      clipName,
      timeSeconds,
      rootVelocity: Object.freeze([...features.rootVelocity]),
      bonePositions: Object.freeze(bonePositions),
      boneRotations: Object.freeze(boneRotations),
      features: immutableFeatures(features),
    });
    this.skeleton = pose.skeleton;
    this.samples.push(sample);
    if (this.samples.length > this.capacity) this.samples.splice(0, this.samples.length - this.capacity);
    return sample;
  }

  latest(count = this.samples.length) {
    if (!Number.isInteger(count) || count < 0) throw new RangeError("pose history count must be non-negative");
    if (count === 0) return [];
    return this.samples.slice(-count);
  }

  clear() {
    this.skeleton = null;
    this.samples = [];
  }
}
