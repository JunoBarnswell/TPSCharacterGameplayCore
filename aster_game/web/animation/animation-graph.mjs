import { evaluateBlendSpace } from "./blend-space.mjs";
import { evaluateAimOffset } from "./aim-offset.mjs";
import { orientationWarpAngle } from "./orientation-warp.mjs";
import { blendWeightedPoses, Pose } from "./pose.mjs";
import { evaluateAimOffsetPose } from "./aim-offset.mjs";
import { blendAnimationLayers } from "./layers.mjs";

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
    phaseProgress: frame.phaseProgress,
    turnDirection: frame.turnDirection,
    turnProgress: frame.turnProgress,
    remainingTurnAngle: frame.remainingTurnAngle,
    actionLayer: frame.actionLayer,
    actionLayers: frame.actionLayers,
  };
}

export function evaluatePoseAnimationGraph(frame, tuning, blendWeightSmoother, poseLibrary, dt) {
  if (!poseLibrary?.locomotionPoses || !poseLibrary?.aimOffsetPoses) {
    throw new TypeError("pose graph requires locomotion and aim pose libraries");
  }
  const semantic = evaluateAnimationGraph(frame, tuning, blendWeightSmoother, dt);
  const locomotionPoses = Object.entries(semantic.locomotion)
    .filter(([, weight]) => weight > 0)
    .map(([name, weight]) => {
      const pose = poseLibrary.locomotionPoses[name];
      if (!(pose instanceof Pose)) throw new RangeError(`locomotion pose '${name}' is missing`);
      return { name, pose, weight };
    });
  const locomotionPose = blendWeightedPoses(locomotionPoses.map(({ pose, weight }) => ({ pose, weight })));
  const aimedPose = evaluateAimOffsetPose(
    locomotionPose,
    poseLibrary.aimOffsetPoses,
    frame.aimYaw,
    frame.aimPitch,
    poseLibrary.aimWeight ?? 1,
    poseLibrary.aimBoneMask ?? null,
  );
  const resolvedLayers = typeof poseLibrary.layersForFrame === "function"
    ? poseLibrary.layersForFrame(frame)
    : poseLibrary.layers ?? [];
  const actionLayers = frame.actionLayers.map((action) => {
    const configured = poseLibrary.actionPoses?.[action];
    if (!configured) throw new RangeError(`action pose '${action}' is missing`);
    return {
      ...configured,
      name: configured.name ?? `action:${action}`,
    };
  });
  const layers = [...resolvedLayers, ...actionLayers];
  return {
    ...semantic,
    pose: blendAnimationLayers(aimedPose, layers),
    selectedLocomotionSamples: locomotionPoses.map(({ name, weight }) => ({ name, weight })),
    activePoseLayers: layers.map(({ name, channel, mode, weight }) => ({ name, channel, mode, weight })),
  };
}
