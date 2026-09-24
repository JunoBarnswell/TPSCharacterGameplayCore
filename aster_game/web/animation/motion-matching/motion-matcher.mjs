import { PoseSearch } from "./pose-search.mjs";
import { extractPoseFeatures } from "./pose-features.mjs";

export class MotionMatcher {
  constructor(poseSearch, { candidateLimit = 5 } = {}) {
    if (!(poseSearch instanceof PoseSearch) || !Number.isInteger(candidateLimit) || candidateLimit < 1) {
      throw new TypeError("motion matcher requires a pose search and positive candidate limit");
    }
    this.poseSearch = poseSearch;
    this.candidateLimit = candidateLimit;
    this.currentCandidate = null;
    this.lastTransitionReason = "not_started";
    this.lastResult = null;
  }

  update({ pose, rootVelocity, trajectory, dt = 1 / 60, previousSample = null }) {
    if (!pose || pose.skeleton !== this.poseSearch.database.skeleton) {
      throw new TypeError("motion matcher input pose must use the pose database skeleton");
    }
    const features = extractPoseFeatures({ pose, rootVelocity, trajectory, previousSample, dt });
    const search = this.poseSearch.search(features, {
      currentCandidateId: this.currentCandidate?.candidate.id ?? null,
      currentClipName: this.currentCandidate?.candidate.clipName ?? null,
      currentTimeSeconds: this.currentCandidate?.candidate.timeSeconds ?? null,
      limit: this.candidateLimit,
    });
    const selected = search.results[0];
    if (!selected) throw new RangeError("motion matching search returned no candidate pose");
    if (!this.currentCandidate) {
      this.lastTransitionReason = "initial_pose_match";
    } else if (selected.candidate.id === this.currentCandidate.candidate.id) {
      this.lastTransitionReason = "current_pose_remains_best";
    } else if (selected.candidate.clipName === this.currentCandidate.candidate.clipName) {
      this.lastTransitionReason = "same_clip_pose_continuity";
    } else {
      this.lastTransitionReason = "lower_weighted_motion_cost";
    }
    this.currentCandidate = selected;
    this.lastResult = Object.freeze({
      selectedPoseId: selected.candidate.id,
      selectedClip: selected.candidate.clipName,
      selectedTimeSeconds: selected.candidate.timeSeconds,
      selectedPose: selected.candidate.pose,
      costs: Object.freeze({
        total: selected.cost,
        pose: selected.poseCost,
        trajectory: selected.trajectoryCost,
        velocity: selected.velocityCost,
        facing: selected.facingCost,
        continuity: selected.continuityCost,
      }),
      candidates: Object.freeze(search.results),
      candidateCount: search.candidateCount,
      transitionReason: this.lastTransitionReason,
      features,
    });
    return this.lastResult;
  }

  reset() {
    this.currentCandidate = null;
    this.lastTransitionReason = "not_started";
    this.lastResult = null;
  }
}
