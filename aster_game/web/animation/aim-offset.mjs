import { blendWeightedPoses } from "./pose.mjs";
import { applyAdditivePose } from "./layers.mjs";

const yawAxis = [-90, 0, 90];
const pitchAxis = [-60, 0, 60];

function bracket(axis, value) {
  if (value <= axis[0]) return [0, 0, 0];
  if (value >= axis.at(-1)) return [axis.length - 2, 1, 1];
  for (let index = 0; index < axis.length - 1; index++) {
    if (value <= axis[index + 1]) {
      const alpha = (value - axis[index]) / (axis[index + 1] - axis[index]);
      return [index, alpha, 0];
    }
  }
  throw new Error("aim offset axis could not be bracketed");
}

function sampleName(pitchIndex, yawIndex) {
  if (pitchIndex === 1 && yawIndex === 1) return "center";
  if (pitchIndex === 2) return yawIndex === 0 ? "up_left" : yawIndex === 1 ? "up" : "up_right";
  if (pitchIndex === 0) return yawIndex === 0 ? "down_left" : yawIndex === 1 ? "down" : "down_right";
  return yawIndex === 0 ? "left" : "right";
}

export function evaluateAimOffset(aimYaw, aimPitch) {
  if (!Number.isFinite(aimYaw) || !Number.isFinite(aimPitch)) {
    throw new TypeError("aim offset angles must be finite");
  }
  const yaw = Math.max(yawAxis[0], Math.min(yawAxis.at(-1), aimYaw));
  const pitch = Math.max(pitchAxis[0], Math.min(pitchAxis.at(-1), aimPitch));
  const [yawIndex, yawAlpha, yawEdge] = bracket(yawAxis, yaw);
  const [pitchIndex, pitchAlpha, pitchEdge] = bracket(pitchAxis, pitch);
  const yawWeights = yawEdge ? [0, 1] : [1 - yawAlpha, yawAlpha];
  const pitchWeights = pitchEdge ? [0, 1] : [1 - pitchAlpha, pitchAlpha];
  const weights = {};
  for (let pitchOffset = 0; pitchOffset <= 1; pitchOffset++) {
    for (let yawOffset = 0; yawOffset <= 1; yawOffset++) {
      const weight = pitchWeights[pitchOffset] * yawWeights[yawOffset];
      if (weight <= 0) continue;
      const name = sampleName(pitchIndex + pitchOffset, yawIndex + yawOffset);
      weights[name] = (weights[name] ?? 0) + weight;
    }
  }
  const total = Object.values(weights).reduce((sum, weight) => sum + weight, 0);
  if (!(total > 0)) throw new Error("aim offset samples produced no pose weight");
  for (const name of Object.keys(weights)) weights[name] /= total;
  return { yaw, pitch, weights };
}

export function evaluateAimOffsetPose(basePose, additiveSamples, aimYaw, aimPitch, weight = 1, boneMask = null) {
  if (!(weight >= 0 && weight <= 1)) throw new RangeError("aim offset weight must be in [0, 1]");
  const aim = evaluateAimOffset(aimYaw, aimPitch);
  const weightedSamples = Object.entries(aim.weights).map(([name, sampleWeight]) => {
    const pose = additiveSamples[name];
    if (!pose) throw new RangeError(`aim offset sample '${name}' is missing`);
    return { pose, weight: sampleWeight };
  });
  const additivePose = blendWeightedPoses(weightedSamples);
  return applyAdditivePose(basePose, additivePose, weight, boneMask);
}
