import { PoseSearch } from "./pose-search.mjs";
import { extractPoseFeatures } from "./pose-features.mjs";

export class MotionMatcher {
  constructor(poseSearch, {
    candidateLimit = 5,
    minimumHoldSeconds = 0.1,
    switchCostThreshold = 0.05,
  } = {}) {
    if (!(poseSearch instanceof PoseSearch) || !Number.isInteger(candidateLimit) || candidateLimit < 1 ||
        !Number.isFinite(minimumHoldSeconds) || minimumHoldSeconds < 0 ||
        !Number.isFinite(switchCostThreshold) || switchCostThreshold < 0) {
      throw new TypeError("motion matcher requires valid search, hold duration and switch threshold");
    }
    this.poseSearch = poseSearch;
    this.candidateLimit = candidateLimit;
    this.minimumHoldSeconds = minimumHoldSeconds;
    this.switchCostThreshold = switchCostThreshold;
    this.currentCandidate = null;
    this.currentHoldSeconds = 0;
    this.playbackTimeSeconds = null;
    this.lastTransitionReason = "not_started";
    this.lastResult = null;
  }

  update({
    pose,
    rootVelocity,
    trajectory,
    dt = 1 / 60,
    previousSample = null,
    contacts = { left: false, right: false },
    forceSwitch = false,
  }) {
    if (!pose || pose.skeleton !== this.poseSearch.database.skeleton) {
      throw new TypeError("motion matcher input pose must use the pose database skeleton");
    }
    if (!(dt > 0) || !Number.isFinite(dt) || typeof forceSwitch !== "boolean") {
      throw new TypeError("motion matcher timestep or transition flag is invalid");
    }
    const features = extractPoseFeatures({
      pose,
      rootVelocity,
      trajectory,
      previousSample,
      contacts,
      dt,
    });
    const search = this.poseSearch.search(features, {
      currentCandidateId: this.currentCandidate?.candidate.id ?? null,
      currentClipName: this.currentCandidate?.candidate.clipName ?? null,
      currentTimeSeconds: this.playbackTimeSeconds,
      limit: this.candidateLimit,
    });
    const best = search.results[0];
    if (!best) throw new RangeError("motion matching search returned no candidate pose");
    let selected = best;
    if (this.currentCandidate) {
      this.currentHoldSeconds += dt;
      const current = search.results.find(({ candidate }) =>
        candidate.id === this.currentCandidate.candidate.id);
      if (!current) throw new RangeError("motion search omitted the active candidate");
      if (best.candidate.id === current.candidate.id) {
        selected = current;
        this.lastTransitionReason = "current_pose_remains_best";
      } else if (best.candidate.clipName === current.candidate.clipName) {
        selected = best;
        this.lastTransitionReason = "same_clip_pose_continuity";
      } else if (!forceSwitch && this.currentHoldSeconds < this.minimumHoldSeconds) {
        selected = current;
        this.lastTransitionReason = "minimum_hold";
      } else if (!forceSwitch && current.cost - best.cost < this.switchCostThreshold) {
        selected = current;
        this.lastTransitionReason = "switch_hysteresis";
      } else {
        this.lastTransitionReason = forceSwitch
          ? "forced_transition"
          : "lower_weighted_motion_cost";
      }
    } else {
      this.lastTransitionReason = "initial_pose_match";
    }
    const clipChanged = !this.currentCandidate ||
      selected.candidate.clipName !== this.currentCandidate.candidate.clipName;
    if (clipChanged) {
      this.currentHoldSeconds = 0;
      this.playbackTimeSeconds = selected.candidate.timeSeconds;
    } else {
      const duration = selected.candidate.metadata?.durationSeconds;
      if (Number.isFinite(duration) && duration > 0) {
        this.playbackTimeSeconds = (this.playbackTimeSeconds + dt) % duration;
      } else {
        this.playbackTimeSeconds = selected.candidate.timeSeconds;
      }
    }
    this.currentCandidate = selected;
    this.lastResult = Object.freeze({
      selectedPoseId: selected.candidate.id,
      selectedClip: selected.candidate.clipName,
      selectedTimeSeconds: selected.candidate.timeSeconds,
      playbackTimeSeconds: this.playbackTimeSeconds,
      selectedPose: selected.candidate.pose,
      costs: Object.freeze({
        total: selected.cost,
        pose: selected.poseCost,
        trajectory: selected.trajectoryCost,
        velocity: selected.velocityCost,
        facing: selected.facingCost,
        contacts: selected.contactCost,
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
    this.currentHoldSeconds = 0;
    this.playbackTimeSeconds = null;
    this.lastTransitionReason = "not_started";
    this.lastResult = null;
  }
}
