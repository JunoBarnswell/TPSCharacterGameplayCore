import { Pose } from "../pose.mjs";
import { flattenPoseFeatureVector, groupPoseFeatures } from "./pose-features.mjs";

function featureVector(candidate) {
  const vector = candidate.features?.vector ?? candidate.featureVector;
  if (!Array.isArray(vector) || vector.length === 0 || !vector.every(Number.isFinite)) {
    throw new TypeError("pose database candidates require finite feature vectors");
  }
  return [...vector];
}

function freezeFeatures(features) {
  const vector = featureVector({ features });
  const canonical = flattenPoseFeatureVector(features);
  if (canonical.length !== vector.length || canonical.some((value, index) => value !== vector[index])) {
    throw new RangeError("pose database feature vector must match its structured features");
  }
  const fieldNames = [
    "rootVelocity", "facing", "contacts", "pelvisPosition", "pelvisVelocity",
    "leftFootPosition", "rightFootPosition", "leftFootVelocity", "rightFootVelocity",
  ];
  const frozen = {};
  for (const name of fieldNames) {
    const value = features[name];
    if (!Array.isArray(value) || !value.every(Number.isFinite)) {
      throw new TypeError(`pose database feature '${name}' must be a finite vector`);
    }
    if (name === "contacts" &&
        (value.length !== 2 || value.some((contact) => contact < 0 || contact > 1))) {
      throw new RangeError("pose database foot contacts must contain two weights in [0, 1]");
    }
    frozen[name] = Object.freeze([...value]);
  }
  if (!Array.isArray(features.trajectory)) {
    throw new TypeError("pose database trajectory features must be an array");
  }
  frozen.trajectory = Object.freeze(features.trajectory.map((sample) => {
    if (!sample || !Array.isArray(sample.position) || !sample.position.every(Number.isFinite) ||
        !Array.isArray(sample.facing) || !sample.facing.every(Number.isFinite) ||
        !Array.isArray(sample.velocity) || !sample.velocity.every(Number.isFinite)) {
      throw new TypeError("pose database trajectory samples must contain finite vectors");
    }
    return Object.freeze({
      position: Object.freeze([...sample.position]),
      facing: Object.freeze([...sample.facing]),
      velocity: Object.freeze([...sample.velocity]),
    });
  }));
  frozen.vector = Object.freeze(vector);
  return Object.freeze(frozen);
}

export class PoseDatabase {
  constructor(candidates = []) {
    if (!Array.isArray(candidates)) throw new TypeError("pose database candidates must be an array");
    this.candidates = [];
    this.ids = new Set();
    this.featureLength = null;
    this.skeleton = null;
    for (const candidate of candidates) this.add(candidate);
    this.normalization = this.computeNormalization();
    this.groupNormalization = Object.freeze(Object.fromEntries(
      ['pose', 'trajectory', 'velocity', 'facing', 'contacts'].map((name) => {
        const vectors = this.candidates.map(({ features }) => groupPoseFeatures(features)[name]);
        const floor = { pose: 0.25, trajectory: 0.5, velocity: 1,
          facing: 0.25, contacts: 0.5 }[name];
        return [name, Object.freeze(Array.from({ length: vectors[0]?.length ?? 0 }, (_, index) => {
          const values = vectors.map((vector) => vector[index]);
          const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
          const variance = values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / values.length;
          return Object.freeze({ mean, scale: Math.max(floor, Math.sqrt(variance)) });
        }))];
      }),
    ));
    Object.freeze(this.candidates);
  }

  computeNormalization(floor = 0.25) {
    if (this.candidates.length === 0) return Object.freeze([]);
    return Object.freeze(Array.from({ length: this.featureLength }, (_, index) => {
      const values = this.candidates.map(({ featureVector }) => featureVector[index]);
      const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
      const variance = values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / values.length;
      return Object.freeze({ mean, scale: Math.max(floor, Math.sqrt(variance)) });
    }));
  }

  add(candidate) {
    if (Object.isFrozen(this.candidates)) throw new RangeError('pose database is frozen after build');
    if (!candidate || typeof candidate.id !== "string" || candidate.id.length === 0 ||
        typeof candidate.clipName !== "string" || candidate.clipName.length === 0 ||
        !Number.isFinite(candidate.timeSeconds) || candidate.timeSeconds < 0 ||
        !(candidate.pose instanceof Pose)) {
      throw new TypeError("pose database entry requires id, clip name, and finite clip time");
    }
    if (this.ids.has(candidate.id)) throw new RangeError(`duplicate pose database id '${candidate.id}'`);
    const clipDurationSeconds = candidate.metadata?.durationSeconds;
    if (clipDurationSeconds !== undefined &&
        (!(clipDurationSeconds > 0) || !Number.isFinite(clipDurationSeconds) ||
         candidate.timeSeconds >= clipDurationSeconds)) {
      throw new RangeError("pose database clip duration must contain the sample time");
    }
    if (this.skeleton !== null && candidate.pose.skeleton !== this.skeleton) {
      throw new TypeError("pose database entries must use one skeleton instance");
    }
    const features = freezeFeatures(candidate.features);
    const vector = features.vector;
    if (this.featureLength !== null && vector.length !== this.featureLength) {
      throw new RangeError("pose database entries must have equal feature vector lengths");
    }
    this.featureLength ??= vector.length;
    this.skeleton ??= candidate.pose.skeleton;
    const stored = Object.freeze({
      ...candidate,
      features,
      featureVector: vector,
      metadata: Object.freeze({ ...(candidate.metadata ?? {}) }),
    });
    this.ids.add(candidate.id);
    this.candidates.push(stored);
    return stored;
  }

  size() {
    return this.candidates.length;
  }
}
