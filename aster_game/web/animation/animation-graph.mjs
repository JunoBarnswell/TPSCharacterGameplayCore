import { evaluateBlendSpace } from "./blend-space.mjs";
import { evaluateAimOffset } from "./aim-offset.mjs";
import { orientationWarpAngle } from "./orientation-warp.mjs";

export function evaluateAnimationGraph(frame, tuning, blendWeightSmoother, dt) {
  const blendWeights = evaluateBlendSpace(frame, tuning.sprint_speed);
  const aim = evaluateAimOffset(frame.aimYaw, frame.aimPitch);
  const transition = blendWeightSmoother.update(blendWeights, dt);
  return {
    locomotion: transition,
    aimOffset: aim,
    hitReaction: frame.actionLayer === "hit_reaction" ? frame.hitStrength : 0,
    orientationWarp: orientationWarpAngle(frame.worldMovementDirection, frame.characterYaw),
    phase: frame.locomotionPhase,
    actionLayer: frame.actionLayer,
  };
}
