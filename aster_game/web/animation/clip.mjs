import { createPose, createTransform, Pose, quaternionSlerp, Skeleton } from "./pose.mjs";

export class Keyframe {
  constructor(timeSeconds, transform) {
    if (!(timeSeconds >= 0) || !Number.isFinite(timeSeconds)) {
      throw new RangeError("keyframe time must be finite and non-negative");
    }
    this.timeSeconds = timeSeconds;
    this.transform = createTransform(transform.translation, transform.rotation, transform.scale);
    Object.freeze(this);
  }
}

export class AnimationTrack {
  constructor(boneIndex, keyframes) {
    if (!Number.isInteger(boneIndex) || boneIndex < 0 || !Array.isArray(keyframes) || keyframes.length === 0) {
      throw new TypeError("animation track requires a valid bone index and keyframes");
    }
    for (let index = 0; index < keyframes.length; index++) {
      if (!(keyframes[index] instanceof Keyframe) ||
          (index > 0 && keyframes[index].timeSeconds <= keyframes[index - 1].timeSeconds)) {
        throw new RangeError("animation track keyframes must be strictly time ordered");
      }
    }
    this.boneIndex = boneIndex;
    this.keyframes = Object.freeze([...keyframes]);
    Object.freeze(this);
  }
}

export class AnimationClip {
  constructor(name, durationSeconds, tracks) {
    if (typeof name !== "string" || name.length === 0 || !(durationSeconds > 0) ||
        !Number.isFinite(durationSeconds) || !Array.isArray(tracks)) {
      throw new TypeError("animation clip requires a name, positive duration, and tracks");
    }
    const bones = new Set();
    for (const track of tracks) {
      if (!(track instanceof AnimationTrack) || bones.has(track.boneIndex) ||
          track.keyframes.at(-1).timeSeconds > durationSeconds) {
        throw new RangeError("animation clip tracks must be unique and fit within its duration");
      }
      bones.add(track.boneIndex);
    }
    this.name = name;
    this.durationSeconds = durationSeconds;
    this.tracks = Object.freeze([...tracks]);
    Object.freeze(this);
  }
}

function sampleTrack(track, timeSeconds) {
  const keyframes = track.keyframes;
  if (timeSeconds <= keyframes[0].timeSeconds) return keyframes[0].transform;
  if (timeSeconds >= keyframes.at(-1).timeSeconds) return keyframes.at(-1).transform;
  let low = 0;
  let high = keyframes.length - 1;
  while (low + 1 < high) {
    const middle = Math.floor((low + high) / 2);
    if (keyframes[middle].timeSeconds <= timeSeconds) low = middle;
    else high = middle;
  }
  const left = keyframes[low];
  const right = keyframes[high];
  const alpha = (timeSeconds - left.timeSeconds) / (right.timeSeconds - left.timeSeconds);
  return createTransform(
    left.transform.translation.map((value, axis) =>
      value + (right.transform.translation[axis] - value) * alpha),
    quaternionSlerp(left.transform.rotation, right.transform.rotation, alpha),
    left.transform.scale.map((value, axis) =>
      value + (right.transform.scale[axis] - value) * alpha),
  );
}

export function sampleAnimationClip(clip, skeleton, timeSeconds) {
  if (!(clip instanceof AnimationClip) || !(skeleton instanceof Skeleton) ||
      !Number.isFinite(timeSeconds)) {
    throw new TypeError("clip sampling requires a clip, skeleton, and finite time");
  }
  const localTransforms = skeleton.bones.map((bone) => bone.bindLocal);
  const sampleTime = Math.max(0, Math.min(clip.durationSeconds, timeSeconds));
  for (const track of clip.tracks) {
    if (track.boneIndex >= skeleton.bones.length) {
      throw new RangeError(`clip '${clip.name}' references a bone outside the skeleton`);
    }
    localTransforms[track.boneIndex] = sampleTrack(track, sampleTime);
  }
  const pose = createPose(skeleton, localTransforms);
  if (!(pose instanceof Pose)) throw new Error("clip sampler failed to construct a pose");
  return pose;
}

export class ClipSampler {
  constructor(clip, skeleton) {
    if (!(clip instanceof AnimationClip) || !(skeleton instanceof Skeleton)) {
      throw new TypeError("clip sampler requires an animation clip and skeleton");
    }
    if (clip.tracks.some((track) => track.boneIndex >= skeleton.bones.length)) {
      throw new RangeError(`clip '${clip.name}' references a bone outside the skeleton`);
    }
    this.clip = clip;
    this.skeleton = skeleton;
  }

  sample(timeSeconds) {
    return sampleAnimationClip(this.clip, this.skeleton, timeSeconds);
  }
}
