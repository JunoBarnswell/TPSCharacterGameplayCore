const IDENTITY_ROTATION = [0, 0, 0, 1];

function vector(values, length, name) {
  if (!Array.isArray(values) || values.length !== length || !values.every(Number.isFinite)) {
    throw new TypeError(`${name} must contain ${length} finite numbers`);
  }
  return [...values];
}

function normalizeQuaternion(values) {
  const quaternion = vector(values, 4, "quaternion");
  const magnitude = Math.hypot(...quaternion);
  if (!(magnitude > 1e-12)) throw new RangeError("quaternion magnitude must be non-zero");
  return quaternion.map((value) => value / magnitude);
}

export function createTransform(
  translation = [0, 0, 0],
  rotation = IDENTITY_ROTATION,
  scale = [1, 1, 1],
) {
  return Object.freeze({
    translation: Object.freeze(vector(translation, 3, "translation")),
    rotation: Object.freeze(normalizeQuaternion(rotation)),
    scale: Object.freeze(vector(scale, 3, "scale")),
  });
}

export class Bone {
  constructor(name, parentIndex, bindLocal = createTransform()) {
    if (typeof name !== "string" || name.length === 0 || !Number.isInteger(parentIndex) || parentIndex < -1) {
      throw new TypeError("bone requires a name and a valid parent index");
    }
    this.name = name;
    this.parentIndex = parentIndex;
    this.bindLocal = createTransform(bindLocal.translation, bindLocal.rotation, bindLocal.scale);
    Object.freeze(this);
  }
}

export class Skeleton {
  constructor(definitions) {
    if (!Array.isArray(definitions) || definitions.length === 0) {
      throw new RangeError("skeleton must contain at least one bone");
    }
    const names = new Set();
    this.bones = definitions.map((definition, index) => {
      if (!definition || typeof definition.name !== "string" || definition.name.length === 0 ||
          names.has(definition.name)) {
        throw new TypeError(`skeleton bone ${index} must have a unique non-empty name`);
      }
      const parentIndex = definition.parentIndex ?? -1;
      if (!Number.isInteger(parentIndex) || parentIndex < -1 || parentIndex >= index) {
        throw new RangeError(`skeleton bone '${definition.name}' must reference an earlier parent`);
      }
      names.add(definition.name);
      return new Bone(definition.name, parentIndex, definition.bindLocal ?? createTransform());
    });
    this.indexByName = new Map(this.bones.map((bone, index) => [bone.name, index]));
    Object.freeze(this.bones);
  }
}

export class Pose {
  constructor(skeleton, localTransforms = skeleton.bones.map((bone) => bone.bindLocal)) {
    if (!(skeleton instanceof Skeleton) || !Array.isArray(localTransforms) ||
        localTransforms.length !== skeleton.bones.length) {
      throw new TypeError("pose transforms must match a valid skeleton");
    }
    this.skeleton = skeleton;
    this.localTransforms = Object.freeze(localTransforms.map((transform) => createTransform(
      transform.translation,
      transform.rotation,
      transform.scale,
    )));
    Object.freeze(this);
  }
}

export function createSkeleton(definitions) {
  return new Skeleton(definitions);
}

export function createPose(skeleton, localTransforms) {
  return new Pose(skeleton, localTransforms);
}

function quaternionMultiply(left, right) {
  const [ax, ay, az, aw] = left;
  const [bx, by, bz, bw] = right;
  return normalizeQuaternion([
    aw * bx + ax * bw + ay * bz - az * by,
    aw * by - ax * bz + ay * bw + az * bx,
    aw * bz + ax * by - ay * bx + az * bw,
    aw * bw - ax * bx - ay * by - az * bz,
  ]);
}

function rotateVector(rotation, value) {
  const [x, y, z, w] = rotation;
  const [vx, vy, vz] = value;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  return [
    vx + w * tx + y * tz - z * ty,
    vy + w * ty + z * tx - x * tz,
    vz + w * tz + x * ty - y * tx,
  ];
}

export function quaternionSlerp(leftInput, rightInput, alphaInput) {
  const left = normalizeQuaternion(leftInput);
  let right = normalizeQuaternion(rightInput);
  const alpha = Math.max(0, Math.min(1, alphaInput));
  let dot = left.reduce((sum, value, index) => sum + value * right[index], 0);
  if (dot < 0) {
    right = right.map((value) => -value);
    dot = -dot;
  }
  if (dot > 0.9995) {
    return normalizeQuaternion(left.map((value, index) => value + (right[index] - value) * alpha));
  }
  const theta = Math.acos(Math.max(-1, Math.min(1, dot)));
  const sine = Math.sin(theta);
  const leftWeight = Math.sin((1 - alpha) * theta) / sine;
  const rightWeight = Math.sin(alpha * theta) / sine;
  return normalizeQuaternion(left.map((value, index) => value * leftWeight + right[index] * rightWeight));
}

function blendTransform(left, right, alpha) {
  return createTransform(
    left.translation.map((value, index) => value + (right.translation[index] - value) * alpha),
    quaternionSlerp(left.rotation, right.rotation, alpha),
    left.scale.map((value, index) => value + (right.scale[index] - value) * alpha),
  );
}

function requireSameSkeleton(poses) {
  if (poses.length === 0 || !(poses[0].pose instanceof Pose)) {
    throw new TypeError("pose blend requires one or more valid poses");
  }
  const skeleton = poses[0].pose.skeleton;
  if (poses.some(({ pose }) => !(pose instanceof Pose) || pose.skeleton !== skeleton)) {
    throw new TypeError("all blended poses must use the same skeleton instance");
  }
  return skeleton;
}

export function blendPoses(left, right, alpha) {
  if (!(left instanceof Pose) || !(right instanceof Pose) || left.skeleton !== right.skeleton) {
    throw new TypeError("pose blend inputs must use the same skeleton instance");
  }
  if (!Number.isFinite(alpha)) throw new TypeError("pose blend alpha must be finite");
  const weight = Math.max(0, Math.min(1, alpha));
  return new Pose(left.skeleton, left.localTransforms.map((transform, index) =>
    blendTransform(transform, right.localTransforms[index], weight)));
}

export function blendWeightedPoses(weightedPoses) {
  if (!Array.isArray(weightedPoses) || weightedPoses.length === 0) {
    throw new RangeError("weighted pose blend requires at least one pose");
  }
  const active = weightedPoses.map(({ pose, weight }) => {
    if (!Number.isFinite(weight) || weight < 0) throw new RangeError("pose weights must be non-negative");
    return { pose, weight };
  }).filter(({ weight }) => weight > 0);
  if (active.length === 0) throw new RangeError("weighted pose blend must have positive total weight");
  const skeleton = requireSameSkeleton(active);
  const total = active.reduce((sum, { weight }) => sum + weight, 0);
  const normalized = active.map(({ pose, weight }) => ({ pose, weight: weight / total }));
  const localTransforms = skeleton.bones.map((_, boneIndex) => {
    const first = normalized[0].pose.localTransforms[boneIndex];
    let accumulatedWeight = normalized[0].weight;
    let rotation = [...first.rotation];
    const translation = first.translation.map((value) => value * accumulatedWeight);
    const scale = first.scale.map((value) => value * accumulatedWeight);
    for (let index = 1; index < normalized.length; index++) {
      const { pose, weight } = normalized[index];
      const transform = pose.localTransforms[boneIndex];
      const combinedWeight = accumulatedWeight + weight;
      rotation = quaternionSlerp(rotation, transform.rotation, weight / combinedWeight);
      for (let axis = 0; axis < 3; axis++) {
        translation[axis] += transform.translation[axis] * weight;
        scale[axis] += transform.scale[axis] * weight;
      }
      accumulatedWeight = combinedWeight;
    }
    return createTransform(
      translation,
      rotation,
      scale,
    );
  });
  return new Pose(skeleton, localTransforms);
}

export function worldTransforms(pose) {
  if (!(pose instanceof Pose)) throw new TypeError("world transforms require a valid pose");
  const worlds = [];
  for (let index = 0; index < pose.localTransforms.length; index++) {
    const local = pose.localTransforms[index];
    const parentIndex = pose.skeleton.bones[index].parentIndex;
    if (parentIndex < 0) {
      worlds.push(createTransform(local.translation, local.rotation, local.scale));
      continue;
    }
    const parent = worlds[parentIndex];
    if (!parent) throw new Error("skeleton world transform order is invalid");
    const scaledTranslation = local.translation.map((value, axis) => value * parent.scale[axis]);
    const rotatedTranslation = rotateVector(parent.rotation, scaledTranslation);
    worlds.push(createTransform(
      parent.translation.map((value, axis) => value + rotatedTranslation[axis]),
      quaternionMultiply(parent.rotation, local.rotation),
      parent.scale.map((value, axis) => value * local.scale[axis]),
    ));
  }
  return worlds;
}
