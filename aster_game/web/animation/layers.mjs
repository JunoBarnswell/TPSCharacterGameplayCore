import { createTransform, Pose, quaternionSlerp } from "./pose.mjs";

export const AnimationLayerChannel = Object.freeze({
  UPPER_BODY_ACTION: "upper_body_action",
  ADDITIVE_REACTION: "additive_reaction",
  FULL_BODY_OVERRIDE: "full_body_override",
  LIFE_OVERRIDE: "life_override",
});

const channelPriority = new Map([
  [AnimationLayerChannel.UPPER_BODY_ACTION, 10],
  [AnimationLayerChannel.ADDITIVE_REACTION, 20],
  [AnimationLayerChannel.FULL_BODY_OVERRIDE, 30],
  [AnimationLayerChannel.LIFE_OVERRIDE, 40],
]);

export class BoneMask {
  constructor(skeleton, weights = {}, defaultWeight = 0) {
    if (!skeleton?.bones || !Number.isFinite(defaultWeight) || defaultWeight < 0 || defaultWeight > 1) {
      throw new TypeError("bone mask requires a skeleton and a default weight in [0, 1]");
    }
    const entries = weights instanceof Map ? [...weights.entries()] : Object.entries(weights);
    this.skeleton = skeleton;
    this.defaultWeight = defaultWeight;
    this.weights = new Map();
    for (const [key, weight] of entries) {
      const index = typeof key === "number" ? key : skeleton.indexByName.get(String(key));
      if (!Number.isInteger(index) || index < 0 || index >= skeleton.bones.length) {
        throw new RangeError(`bone mask references unknown bone '${key}'`);
      }
      if (!Number.isFinite(weight) || weight < 0 || weight > 1) {
        throw new RangeError(`bone mask weight for '${key}' must be in [0, 1]`);
      }
      this.weights.set(index, weight);
    }
  }

  static fullBody(skeleton) {
    return new BoneMask(skeleton, {}, 1);
  }

  static fromNames(skeleton, names, defaultWeight = 0) {
    if (!Array.isArray(names) || names.length === 0) {
      throw new RangeError("bone mask requires at least one bone name");
    }
    return new BoneMask(skeleton, Object.fromEntries(names.map((name) => [name, 1])), defaultWeight);
  }

  weightFor(boneIndex) {
    if (!Number.isInteger(boneIndex) || boneIndex < 0 || boneIndex >= this.skeleton.bones.length) {
      throw new RangeError("bone mask query is outside its skeleton");
    }
    return this.weights.get(boneIndex) ?? this.defaultWeight;
  }
}

function validateLayerPose(basePose, layer) {
  if (!(layer.pose instanceof Pose) || layer.pose.skeleton !== basePose.skeleton) {
    throw new TypeError(`animation layer '${layer.name}' must use the base pose skeleton`);
  }
  if (layer.boneMask && layer.boneMask.skeleton !== basePose.skeleton) {
    throw new TypeError(`animation layer '${layer.name}' bone mask uses a different skeleton`);
  }
  if (!Number.isFinite(layer.weight) || layer.weight < 0 || layer.weight > 1) {
    throw new RangeError(`animation layer '${layer.name}' weight must be in [0, 1]`);
  }
}

function applyLayer(basePose, layer) {
  validateLayerPose(basePose, layer);
  const transforms = basePose.localTransforms.map((base, index) => {
    const weight = layer.weight * (layer.boneMask?.weightFor(index) ?? 1);
    if (weight <= 0) return base;
    const source = layer.pose.localTransforms[index];
    if (layer.mode === "override") {
      return createTransform(
        base.translation.map((value, axis) => value + (source.translation[axis] - value) * weight),
        quaternionSlerp(base.rotation, source.rotation, weight),
        base.scale.map((value, axis) => value + (source.scale[axis] - value) * weight),
      );
    }
    if (layer.mode !== "additive") throw new TypeError(`unsupported animation blend mode '${layer.mode}'`);
    const identity = [0, 0, 0, 1];
    const additiveRotation = quaternionSlerp(identity, source.rotation, weight);
    const [ax, ay, az, aw] = base.rotation;
    const [bx, by, bz, bw] = additiveRotation;
    const rotation = [
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
    ];
    return createTransform(
      base.translation.map((value, axis) => value + source.translation[axis] * weight),
      rotation,
      base.scale.map((value, axis) => value * (1 + (source.scale[axis] - 1) * weight)),
    );
  });
  return new Pose(basePose.skeleton, transforms);
}

export function applyAdditivePose(basePose, additivePose, weight = 1, boneMask = null) {
  return applyLayer(basePose, {
    name: "additive_pose",
    channel: AnimationLayerChannel.ADDITIVE_REACTION,
    mode: "additive",
    pose: additivePose,
    weight,
    boneMask,
  });
}

export function blendAnimationLayers(basePose, layers) {
  if (!(basePose instanceof Pose) || !Array.isArray(layers)) {
    throw new TypeError("animation layer composition requires a base pose and layer list");
  }
  const names = new Set();
  const ordered = layers.map((layer, insertionOrder) => {
    if (!layer || typeof layer.name !== "string" || layer.name.length === 0 || names.has(layer.name)) {
      throw new TypeError("animation layer names must be unique and non-empty");
    }
    if (!channelPriority.has(layer.channel)) throw new TypeError(`unknown animation layer channel '${layer.channel}'`);
    names.add(layer.name);
    return { ...layer, insertionOrder };
  }).sort((left, right) => channelPriority.get(left.channel) - channelPriority.get(right.channel) ||
    left.insertionOrder - right.insertionOrder);
  return ordered.reduce((pose, layer) => applyLayer(pose, layer), basePose);
}

export class CharacterPoseLayerStack {
  constructor() {
    this.layers = new Map();
  }

  set(name, { channel, pose, mode, weight = 1, boneMask = null }) {
    if (typeof name !== "string" || name.length === 0 || !channelPriority.has(channel)) {
      throw new TypeError("layer stack entry requires a name and a supported channel");
    }
    if (mode !== "override" && mode !== "additive") {
      throw new TypeError("layer stack blend mode must be override or additive");
    }
    this.layers.set(name, { name, channel, pose, mode, weight, boneMask });
  }

  remove(name) {
    return this.layers.delete(name);
  }

  clear() {
    this.layers.clear();
  }

  activeLayers() {
    return [...this.layers.values()].filter(({ weight }) => weight > 0).map(({ name, channel, mode, weight }) => ({
      name, channel, mode, weight,
    }));
  }

  compose(basePose) {
    return blendAnimationLayers(basePose, [...this.layers.values()]);
  }
}
