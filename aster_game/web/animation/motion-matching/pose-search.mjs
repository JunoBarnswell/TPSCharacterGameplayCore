import { PoseDatabase } from "./pose-database.mjs";
import { flattenPoseFeatureVector, groupPoseFeatures } from "./pose-features.mjs";

export const defaultPoseSearchWeights = Object.freeze({
  pose: 1.0,
  trajectory: 1.0,
  velocity: 0.5,
  facing: 0.5,
  contacts: 0.35,
  continuity: 0.25,
});

const normalizationFloors = Object.freeze({
  pose: 0.25,
  trajectory: 0.5,
  velocity: 1,
  facing: 0.25,
  contacts: 0.5,
});

function normalizationScale(vectors, floor) {
  const values = vectors.flat();
  if (values.length === 0) return floor;
  const mean = values.reduce((sum, value) => sum + value, 0) / values.length;
  const variance = values.reduce((sum, value) => sum + (value - mean) ** 2, 0) / values.length;
  return Math.max(floor, Math.sqrt(variance));
}

function normalizedRmsDistance(left, right, name, scale) {
  if (!Array.isArray(left) || !Array.isArray(right) || left.length !== right.length ||
      !left.every(Number.isFinite) || !right.every(Number.isFinite)) {
    throw new RangeError(`pose search feature '${name}' is missing or has mismatched dimensions`);
  }
  if (left.length === 0) return 0;
  return Math.sqrt(left.reduce((sum, value, index) =>
    sum + ((value - right[index]) / (Array.isArray(scale) ? scale[index] : scale)) ** 2, 0) / left.length);
}

function queryFeatures(features) {
  if (!features || !Array.isArray(features.rootVelocity) || !Array.isArray(features.facing) ||
      !Array.isArray(features.contacts) ||
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
    if (!(database instanceof PoseDatabase) || database.size() === 0 ||
        !weights || typeof weights !== "object" ||
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
    this.groupedCandidates = database.candidates.map(({ features }) =>
      groupPoseFeatures(queryFeatures(features)));
    const groups = this.groupedCandidates;
    this.normalizationScales = Object.freeze(Object.fromEntries(
      Object.keys(normalizationFloors).map((name) => [
        name,
        normalizationScale(groups.map((group) => group[name]), normalizationFloors[name]),
      ]),
    ));
    this.dimensionScales = Object.freeze(Object.fromEntries(Object.entries(database.groupNormalization)
      .map(([name, dimensions]) => [name, Object.freeze(dimensions.map(({ scale }) => scale))])));
  }

  search(features, { currentCandidateId = null, currentClipName = null, currentTimeSeconds = null,
    limit = 5, tag = null, clipNames = null } = {}) {
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
    const queryGroups = groupPoseFeatures(query);
    const ranked = this.database.candidates.flatMap((candidate, index) => {
      if ((tag !== null && candidate.metadata.tag !== tag) ||
          (clipNames !== null && !clipNames.includes(candidate.clipName))) return [];
      const candidateGroups = this.groupedCandidates[index];
      const poseCost = normalizedRmsDistance(
        queryGroups.pose, candidateGroups.pose, "pose", this.dimensionScales.pose,
      );
      const trajectoryCost = normalizedRmsDistance(
        queryGroups.trajectory, candidateGroups.trajectory,
        "trajectory", this.dimensionScales.trajectory,
      );
      const velocityCost = normalizedRmsDistance(
        queryGroups.velocity, candidateGroups.velocity,
        "velocity", this.dimensionScales.velocity,
      );
      const facingCost = normalizedRmsDistance(
        queryGroups.facing, candidateGroups.facing, "facing", this.dimensionScales.facing,
      );
      const contactCost = normalizedRmsDistance(
        queryGroups.contacts, candidateGroups.contacts,
        "contacts", this.dimensionScales.contacts,
      );
      const clipDistance = candidate.clipName === currentClipName &&
        Number.isFinite(currentTimeSeconds)
        ? Math.abs(candidate.timeSeconds - currentTimeSeconds)
        : Infinity;
      const clipDuration = candidate.metadata?.durationSeconds;
      const wrappedClipDistance = Number.isFinite(clipDuration) && clipDuration > 0
        ? Math.min(clipDistance % clipDuration, clipDuration - clipDistance % clipDuration)
        : clipDistance;
      const continuityCost = currentCandidateId === null && currentClipName === null
        ? 0
        : candidate.id === currentCandidateId
          ? 0
          : Number.isFinite(wrappedClipDistance)
            ? Math.min(1, wrappedClipDistance)
            : 1;
      const cost = poseCost * this.weights.pose +
        trajectoryCost * this.weights.trajectory +
        velocityCost * this.weights.velocity +
        facingCost * this.weights.facing +
        contactCost * this.weights.contacts +
        continuityCost * this.weights.continuity;
      return [{
        candidate,
        cost,
        poseCost,
        trajectoryCost,
        velocityCost,
        facingCost,
        contactCost,
        continuityCost,
      }];
    }).sort((left, right) => left.cost - right.cost || left.candidate.id.localeCompare(right.candidate.id));
    const results = ranked.slice(0, limit);
    if (currentCandidateId !== null && !results.some(({ candidate }) => candidate.id === currentCandidateId)) {
      const current = ranked.find(({ candidate }) => candidate.id === currentCandidateId);
      if (current) results.push(current);
    }
    return {
      candidateCount: ranked.length,
      results,
      normalizationScales: this.normalizationScales,
    };
  }
}
