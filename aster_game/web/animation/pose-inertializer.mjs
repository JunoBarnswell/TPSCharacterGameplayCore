import {
  createTransform,
  Pose,
  quaternionConjugate,
  quaternionMultiply,
  quaternionSlerp,
} from "./pose.mjs";

function dampingFactor(elapsedSeconds, halfLifeSeconds) {
  const omega = 1.67834699001666 / halfLifeSeconds;
  const scaledTime = omega * elapsedSeconds;
  return (1 + scaledTime) * Math.exp(-scaledTime);
}

export class PoseInertializer {
  constructor(halfLifeSeconds = 0.12) {
    if (!(halfLifeSeconds > 0)) throw new RangeError("pose inertialization half-life must be positive");
    this.halfLifeSeconds = halfLifeSeconds;
    this.skeleton = null;
    this.offsets = null;
  }

  begin(currentPose, targetPose) {
    if (!(currentPose instanceof Pose) || !(targetPose instanceof Pose) ||
        currentPose.skeleton !== targetPose.skeleton) {
      throw new TypeError("pose inertialization requires poses on the same skeleton");
    }
    this.skeleton = currentPose.skeleton;
    this.offsets = currentPose.localTransforms.map((current, index) => {
      const target = targetPose.localTransforms[index];
      return {
        translation: current.translation.map((value, axis) => value - target.translation[axis]),
        scale: current.scale.map((value, axis) => value - target.scale[axis]),
        rotation: quaternionMultiply(current.rotation, quaternionConjugate(target.rotation)),
      };
    });
  }

  sample(targetPose, elapsedSeconds) {
    if (!(targetPose instanceof Pose) || targetPose.skeleton !== this.skeleton || !this.offsets) {
      throw new TypeError("pose inertializer must begin before sampling a matching target pose");
    }
    if (!(elapsedSeconds >= 0) || !Number.isFinite(elapsedSeconds)) {
      throw new RangeError("pose inertialization time must be finite and non-negative");
    }
    const weight = dampingFactor(elapsedSeconds, this.halfLifeSeconds);
    const identity = [0, 0, 0, 1];
    const transforms = targetPose.localTransforms.map((target, index) => {
      const offset = this.offsets[index];
      const rotationOffset = quaternionSlerp(identity, offset.rotation, weight);
      return createTransform(
        target.translation.map((value, axis) => value + offset.translation[axis] * weight),
        quaternionMultiply(rotationOffset, target.rotation),
        target.scale.map((value, axis) => value + offset.scale[axis] * weight),
      );
    });
    return new Pose(this.skeleton, transforms);
  }

  clear() {
    this.skeleton = null;
    this.offsets = null;
  }
}
