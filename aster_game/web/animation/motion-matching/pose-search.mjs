import { PoseDatabase } from "./pose-database.mjs";
import { flattenPoseFeatureVector } from "./pose-features.mjs";

export const defaultPoseSearchWeights = Object.freeze({
  pose: 1.0,
  trajectory: 1.0,
  velocity: 0.5,
  facing: 0.5,
  continuity: 0.25,
});

function rmsDistance(left, right, name) {
  if (!Array.isArray(left) || !Array.isArray(right) || left.length !== right.length ||
      !left.every(Number.isFinite) || !right.every(Number.isFinite)) {
    throw new RangeError(`pose search feature '${name}' is missing or has mismatched dimensions`);
  }
  if (left.length === 0) return 0;
  const squared = left.reduce((sum, value, index) => sum + (value - right[index]) ** 2, 0);
  return Math.sqrt(squared / left.length);
}

function flatten(values) {
  return values.flatMap((value) => [...value]);
}

function queryFeatures(features) {
  if (!features || !Array.isArray(features.rootVelocity) || !Array.isArray(features.facing) ||
      !Array.isArray(features.pelvisPosition) || !Array.isArray(features.pelvisVelocity) ||
      !Array.isArray(features.leftFootPosition) || !Array.isArray(features.rightFootPosition) ||
      !Array.isArray(features.leftFootVelocity) || !Array.isArray(features.rightFootVelocity) ||
      !Array.isArray(features.trajectory)) {
    throw new TypeError("pose search requires structured pose and trajectory features");
  }
  return features;
}

export class PoseSearch {
  constructor(database, weights = defaultPoseSearchWeights) {
    if (!(database instanceof PoseDatabase) || !weights || typeof weights !== "object" ||
        Array.isArray(weights)) {
      throw new TypeError("pose search requires a database and weight object");
    }
    const unknownWeights = Object.keys(weights).filter((name) => !Object.hasOwn(defaultPoseSearchWeights, name));
    if (unknownWeights.length) throw new RangeError(`unknown pose search cost '${unknownWeights[0]}'`);
    const normalized = { ...defaultPoseSearchWeights, ...weights };
    if (Object.values(normalized).some((weight) => !Number.isFinite(weight) || weight < 0) ||
        Object.values(normalized).every((weight) => weight === 0)) {
      throw new RangeError("pose search cost weights must be finite, non-negative, and non-zero");
    }
    this.database = database;
    this.weights = Object.freeze(normalized);
  }

  search(features, { currentCandidateId = null, currentClipName = null, currentTimeSeconds = null, limit = 5 } = {}) {
    const query = queryFeatures(features);
    if (!Number.isInteger(limit) || limit < 1) throw new RangeError("pose search result limit must be positive");
    const queryVector = query.vector;
    if (!Array.isArray(queryVector) || queryVector.length !== this.database.featureLength ||
        !queryVector.every(Number.isFinite)) {
      throw new RangeError("pose search query vector does not match the database feature schema");
    }
    const canonicalQueryVector = flattenPoseFeatureVector(query);
    if (canonicalQueryVector.length !== queryVector.length ||
        canonicalQueryVector.some((value, index) => value !== queryVector[index])) {
      throw new RangeError("pose search feature vector does not match its structured features");
    }
    const queryPose = flatten([query.pelvisPosition, query.leftFootPosition, query.rightFootPosition]);
    const queryTrajectory = flatten(query.trajectory.map(({ position }) => position));
    const queryVelocity = flatten([
      query.rootVelocity,
      query.pelvisVelocity,
      query.leftFootVelocity,
      query.rightFootVelocity,
      ...query.trajectory.map(({ velocity }) => velocity),
    ]);
    const queryFacing = flatten([query.facing, ...query.trajectory.map(({ facing }) => facing)]);
    const ranked = this.database.candidates.map((candidate) => {
      const candidateFeatures = queryFeatures(candidate.features);
      const poseCost = rmsDistance(
        queryPose,
        flatten([candidateFeatures.pelvisPosition, candidateFeatures.leftFootPosition, candidateFeatures.rightFootPosition]),
        "pose",
      );
      const trajectoryCost = rmsDistance(
        queryTrajectory,
        flatten(candidateFeatures.trajectory.map(({ position }) => position)),
        "trajectory",
      );
      const velocityCost = rmsDistance(
        queryVelocity,
        flatten([
          candidateFeatures.rootVelocity,
          candidateFeatures.pelvisVelocity,
          candidateFeatures.leftFootVelocity,
          candidateFeatures.rightFootVelocity,
          ...candidateFeatures.trajectory.map(({ velocity }) => velocity),
        ]),
        "velocity",
      );
      const facingCost = rmsDistance(
        queryFacing,
        flatten([candidateFeatures.facing, ...candidateFeatures.trajectory.map(({ facing }) => facing)]),
        "facing",
      );
      const continuityCost = currentCandidateId === null && currentClipName === null
        ? 0
        : candidate.id === currentCandidateId
          ? 0
          : candidate.clipName === currentClipName && Number.isFinite(currentTimeSeconds)
            ? Math.min(1, Math.abs(candidate.timeSeconds - currentTimeSeconds))
            : 1;
      const cost = poseCost * this.weights.pose +
        trajectoryCost * this.weights.trajectory +
        velocityCost * this.weights.velocity +
        facingCost * this.weights.facing +
        continuityCost * this.weights.continuity;
      return { candidate, cost, poseCost, trajectoryCost, velocityCost, facingCost, continuityCost };
    }).sort((left, right) => left.cost - right.cost || left.candidate.id.localeCompare(right.candidate.id));
    return { candidateCount: ranked.length, results: ranked.slice(0, limit) };
  }
}
