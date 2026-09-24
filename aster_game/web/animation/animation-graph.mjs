import { evaluateBlendSpace } from "./blend-space.mjs";
import { evaluateAimOffset } from "./aim-offset.mjs";
import { inertialize } from "./inertialization.mjs";
import { orientationWarpAngle } from "./orientation-warp.mjs";

export function evaluateAnimationGraph(frame, tuning, inertializer, dt) {
  const blendWeights = evaluateBlendSpace(frame, tuning.sprint_speed);
  const aim = evaluateAimOffset(frame.aimYaw, frame.aimPitch);
  const transition = inertialize(inertializer, "locomotion", blendWeights, dt, 0.12);
  return {
    locomotion: transition,
    aimOffset: aim,
    hitReaction: frame.actionLayer === "hit_reaction" ? frame.hitStrength : 0,
    orientationWarp: orientationWarpAngle(frame.worldMovementDirection, frame.characterYaw),
    phase: frame.locomotionPhase,
    actionLayer: frame.actionLayer,
  };
}
