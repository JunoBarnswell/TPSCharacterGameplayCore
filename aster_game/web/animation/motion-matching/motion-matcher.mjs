import { PoseSearch } from "./pose-search.mjs";
import { extractPoseFeatures } from "./pose-features.mjs";
import { ClipSampler } from "../clip.mjs";

export function synchronizeMarkerTime(source, sourceTime, destination) {
  const oldMarkers = source?.metadata?.markers;
  const newMarkers = destination?.metadata?.markers;
  const oldDuration = source?.metadata?.durationSeconds;
  const newDuration = destination?.metadata?.durationSeconds;
  if (!oldMarkers?.length || !newMarkers?.length || !(oldDuration > 0) || !(newDuration > 0)) {
    return destination.timeSeconds;
  }
  const phase = (sourceTime / oldDuration % 1 + 1) % 1;
  const ordered = [...oldMarkers].sort((a, b) => a.phase - b.phase);
  const index = ordered.findLastIndex((marker) => marker.phase <= phase);
  const start = ordered[Math.max(index, 0)];
  const end = ordered[(Math.max(index, 0) + 1) % ordered.length];
  const width = (end.phase - start.phase + 1) % 1 || 1;
  const progress = ((phase - start.phase + 1) % 1) / width;
  const targetStart = newMarkers.find((marker) => marker.name === start.name);
  const targetEnd = newMarkers.find((marker) => marker.name === end.name);
  if (!targetStart || !targetEnd) return destination.timeSeconds;
  return ((targetStart.phase + progress * ((targetEnd.phase - targetStart.phase + 1) % 1 || 1)) % 1) * newDuration;
}

export class MotionMatcher {
  constructor(poseSearch, {
    candidateLimit = 5,
    minimumHoldSeconds = 0.1,
    switchCostThreshold = 0.05,
    clips = null,
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
    this.samplers = clips && new Map(Object.entries(clips).map(([name, clip]) => [
      name, new ClipSampler(clip, poseSearch.database.skeleton),
    ]));
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
    tag = null,
    clipNames = null,
    playbackRate = 1,
  }) {
    if (!pose || pose.skeleton !== this.poseSearch.database.skeleton) {
      throw new TypeError("motion matcher input pose must use the pose database skeleton");
    }
    if (!(dt > 0) || !Number.isFinite(dt) || typeof forceSwitch !== "boolean" ||
        !Number.isFinite(playbackRate) || playbackRate < 0) {
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
    const currentClip = this.currentCandidate?.candidate.clipName;
    const currentDuration = this.currentCandidate?.candidate.metadata?.durationSeconds;
    const continuingTime = currentDuration
      ? (this.playbackTimeSeconds + dt * playbackRate) % currentDuration : this.playbackTimeSeconds;
    const continuingCandidate = currentClip ? this.poseSearch.database.candidates
      .filter((candidate) => candidate.clipName === currentClip)
      .sort((a, b) => Math.abs(a.timeSeconds - continuingTime) -
        Math.abs(b.timeSeconds - continuingTime))[0] : null;
    const search = this.poseSearch.search(features, {
      currentCandidateId: continuingCandidate?.id ?? null,
      currentClipName: this.currentCandidate?.candidate.clipName ?? null,
      currentTimeSeconds: continuingTime,
      limit: this.candidateLimit,
      tag,
      clipNames,
    });
    const best = search.results[0];
    if (!best) throw new RangeError("motion matching search returned no candidate pose");
    let selected = best;
    if (this.currentCandidate) {
      this.currentHoldSeconds += dt;
      const current = search.results.find(({ candidate }) =>
        candidate.id === continuingCandidate?.id);
      if (!current) {
        selected = best;
        this.lastTransitionReason = "source_tag_changed";
      } else {
        if (best.candidate.id === current.candidate.id) {
          selected = current;
          this.lastTransitionReason = "current_pose_remains_best";
        } else if (best.candidate.clipName === current.candidate.clipName) {
          const timeGap = Math.abs(best.candidate.timeSeconds - continuingTime);
          const shouldJump = timeGap > 0.15 && current.cost - best.cost > this.switchCostThreshold &&
            (forceSwitch || this.currentHoldSeconds >= this.minimumHoldSeconds);
          selected = shouldJump ? best : current;
          this.lastTransitionReason = shouldJump ? "same_clip_pose_jump" : "same_clip_pose_continuity";
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
      }
    } else {
      this.lastTransitionReason = "initial_pose_match";
    }
    const switched = !this.currentCandidate ||
      selected.candidate.clipName !== this.currentCandidate.candidate.clipName ||
      this.lastTransitionReason === "same_clip_pose_jump";
    if (switched) {
      const previousCandidate = this.currentCandidate?.candidate;
      const previousTime = this.playbackTimeSeconds;
      this.currentHoldSeconds = 0;
      this.playbackTimeSeconds = previousCandidate?.metadata?.tag === "grounded" &&
        selected.candidate.metadata?.tag === "grounded"
        ? synchronizeMarkerTime(previousCandidate, previousTime, selected.candidate)
        : selected.candidate.timeSeconds;
    } else {
      const duration = selected.candidate.metadata?.durationSeconds;
      if (Number.isFinite(duration) && duration > 0) {
        this.playbackTimeSeconds = (this.playbackTimeSeconds + dt * playbackRate) % duration;
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
      selectedPose: this.samplers?.get(selected.candidate.clipName)?.sample(this.playbackTimeSeconds) ??
        selected.candidate.pose,
      jumped: switched,
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
