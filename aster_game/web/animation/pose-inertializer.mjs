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

function rotationVector(q) {
  const sign = q[3] < 0 ? -1 : 1;
  const length = Math.hypot(q[0], q[1], q[2]);
  if (length < 1e-10) return [0, 0, 0];
  const angle = 2 * Math.atan2(length, Math.abs(q[3]));
  return q.slice(0, 3).map((component) => sign * component * angle / length);
}

function rotationFromVector(vector) {
  const angle = Math.hypot(...vector);
  if (angle < 1e-10) return [0, 0, 0, 1];
  return [...vector.map((value) => value * Math.sin(angle / 2) / angle), Math.cos(angle / 2)];
}

export class PoseInertializer {
  constructor(halfLifeSeconds = 0.12, {
    maxLinearAcceleration = 50, maxAngularAcceleration = 60,
  } = {}) {
    if (!(halfLifeSeconds > 0)) throw new RangeError("pose inertialization half-life must be positive");
    if (!(maxLinearAcceleration > 0) || !(maxAngularAcceleration > 0)) {
      throw new RangeError('inertial acceleration bounds must be positive');
    }
    this.halfLifeSeconds = halfLifeSeconds;
    this.maxLinearAcceleration = maxLinearAcceleration;
    this.maxAngularAcceleration = maxAngularAcceleration;
    this.skeleton = null;
    this.offsets = null;
  }

  begin(currentPose, targetPose, { previousOutput = null, previousTarget = null, dt = null } = {}) {
    if (!(currentPose instanceof Pose) || !(targetPose instanceof Pose) ||
        currentPose.skeleton !== targetPose.skeleton) {
      throw new TypeError("pose inertialization requires poses on the same skeleton");
    }
    this.skeleton = currentPose.skeleton;
    const hasDerivative = previousOutput?.skeleton === this.skeleton &&
      previousTarget?.skeleton === this.skeleton && dt > 0;
    this.offsets = currentPose.localTransforms.map((current, index) => {
      const target = targetPose.localTransforms[index];
      const previous = previousOutput?.localTransforms[index];
      const incoming = previousTarget?.localTransforms[index];
      const rotation = quaternionMultiply(current.rotation, quaternionConjugate(target.rotation));
      const previousRotationOffset = hasDerivative
        ? quaternionMultiply(previous.rotation, quaternionConjugate(incoming.rotation)) : null;
      return {
        translation: current.translation.map((value, axis) => value - target.translation[axis]),
        scale: current.scale.map((value, axis) => value - target.scale[axis]),
        rotation,
        rotationVector: rotationVector(rotation),
        translationVelocity: hasDerivative ? current.translation.map((value, axis) =>
          ((value - previous.translation[axis]) - (target.translation[axis] - incoming.translation[axis])) / dt) : null,
        scaleVelocity: hasDerivative ? current.scale.map((value, axis) =>
          ((value - previous.scale[axis]) - (target.scale[axis] - incoming.scale[axis])) / dt) : null,
        angularVelocity: hasDerivative ? rotationVector(quaternionMultiply(rotation,
          quaternionConjugate(previousRotationOffset))).map((value) => value / dt) : null,
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
    const omega = 1.67834699001666 / this.halfLifeSeconds;
    const damp = (offset, velocity, acceleration) => {
      if (velocity === null) return offset * weight;
      // Bound the transient acceleration caused by a large velocity mismatch.
      const boundedOmega = Math.min(omega,
        Math.sqrt(acceleration / (2 * Math.max(Math.abs(offset), 1e-8))),
        acceleration / (4 * Math.max(Math.abs(velocity), 1e-8)));
      return (offset + (velocity + boundedOmega * offset) * elapsedSeconds) *
        Math.exp(-boundedOmega * elapsedSeconds);
    };
    const identity = [0, 0, 0, 1];
    const transforms = targetPose.localTransforms.map((target, index) => {
      const offset = this.offsets[index];
      const rotationOffset = offset.angularVelocity
        ? rotationFromVector(offset.rotationVector.map((value, axis) =>
          damp(value, offset.angularVelocity[axis], this.maxAngularAcceleration)))
        : quaternionSlerp(identity, offset.rotation, weight);
      return createTransform(
        target.translation.map((value, axis) => value + damp(offset.translation[axis],
          offset.translationVelocity?.[axis] ?? null, this.maxLinearAcceleration)),
        quaternionMultiply(rotationOffset, target.rotation),
        target.scale.map((value, axis) => value + damp(offset.scale[axis],
          offset.scaleVelocity?.[axis] ?? null, this.maxLinearAcceleration)),
      );
    });
    return new Pose(this.skeleton, transforms);
  }

  clear() {
    this.skeleton = null;
    this.offsets = null;
  }
}
