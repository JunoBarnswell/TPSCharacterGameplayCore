import { AnimationClip, AnimationTrack, Keyframe, sampleAnimationClip } from './clip.mjs';
import { createSkeleton, createTransform } from './pose.mjs';
import { PoseHistory } from './motion-matching/pose-history.mjs';
import { PoseDatabase } from './motion-matching/pose-database.mjs';
import { locomotionTuning } from '../motion/locomotion-tuning.mjs';

export const syntheticRig = createSkeleton([
  { name: 'root', parentIndex: -1 },
  { name: 'pelvis', parentIndex: 0 },
  { name: 'spine', parentIndex: 1, bindLocal: createTransform([0, 0.3, 0]) },
  { name: 'left_thigh', parentIndex: 1, bindLocal: createTransform([-0.2, -0.08, 0]) },
  { name: 'left_calf', parentIndex: 3, bindLocal: createTransform([0, -0.44, 0]) },
  { name: 'left_foot', parentIndex: 4, bindLocal: createTransform([0, -0.4, 0]) },
  { name: 'right_thigh', parentIndex: 1, bindLocal: createTransform([0.2, -0.08, 0]) },
  { name: 'right_calf', parentIndex: 6, bindLocal: createTransform([0, -0.44, 0]) },
  { name: 'right_foot', parentIndex: 7, bindLocal: createTransform([0, -0.4, 0]) },
  { name: 'chest', parentIndex: 2, bindLocal: createTransform([0, 0.3, 0]) },
  { name: 'head', parentIndex: 9, bindLocal: createTransform([0, 0.3, 0]) },
]);

export const phaseMarkers = Object.freeze(locomotionTuning.markers);
const directions = [
  ['forward', 0], ['forward_right', 45], ['right', 90], ['backward_right', 135],
  ['backward', 180], ['backward_left', -135], ['left', -90], ['forward_left', -45],
];
const definitions = [
  { name: 'idle', speed: 0, stride: 0, direction: 0, tag: 'grounded' },
  ...Object.entries(locomotionTuning.gaits).map(([gait, data]) =>
    [gait, data.reference_speed, data.stride_length / (2 * Math.PI)])
    .flatMap(([gait, speed, stride]) => directions.map(([name, direction]) => ({
      name: gait === 'sprint' && name === 'forward' ? 'sprint' : `${gait}_${name}`,
      speed, stride, direction, tag: 'grounded', gait,
    }))),
  ...['pivot_reverse', 'turn_left_90', 'turn_right_90'].map((name) => ({
    name, speed: 0, stride: 0.18, direction: 0, tag: 'turn',
  })),
  ...['jump_start', 'rising', 'apex', 'falling', 'soft_land', 'normal_land', 'heavy_land']
    .map((name) => ({ name, speed: 0, stride: 0, direction: 0,
      tag: ['soft_land', 'normal_land', 'heavy_land'].includes(name) ? 'landing' : 'airborne' })),
];

function makeClip(definition) {
  const { name, stride, tag } = definition;
  const gaitTuning = locomotionTuning.gaits[definition.gait];
  const duration = gaitTuning
    ? gaitTuning.stride_length / gaitTuning.reference_speed
    : tag === 'grounded' ? 1 : 0.6;
  const count = 16;
  const frames = Array.from({ length: count + 1 }, (_, index) => {
    const phase = index / count;
    const cycle = phase * 2 * Math.PI;
    const swing = tag === 'grounded' || tag === 'turn' ? Math.sin(cycle) : 0;
    const airborne = name === 'jump_start' ? 0.08 * phase :
      name === 'rising' ? 0.2 * phase : name === 'apex' ? 0.18 :
      name === 'falling' ? 0.12 * (1 - phase) : 0;
    const land = tag === 'landing'
      ? ({ soft_land: 0.08, normal_land: 0.17, heavy_land: 0.28 })[name] *
        Math.sin(phase * Math.PI) : 0;
    const turn = name === 'pivot_reverse' ? 110 * phase :
      name === 'turn_left_90' ? -70 * phase :
      name === 'turn_right_90' ? 70 * phase : definition.direction * 0.08;
    return [
      createTransform(), createTransform([0, airborne - land + Math.abs(swing) * 0.015, 0],
        [0, Math.sin(turn * Math.PI / 360), 0, Math.cos(turn * Math.PI / 360)]),
      createTransform([0, 0.3, 0]),
      createTransform([-0.2, -0.08, 0], [Math.sin(swing * 0.02), 0, 0, Math.cos(swing * 0.02)]),
      createTransform([0, -0.44, 0]),
      createTransform([0, -0.4 + Math.max(0, -swing) * 0.08, -stride * swing]),
      createTransform([0.2, -0.08, 0], [-Math.sin(swing * 0.02), 0, 0, Math.cos(swing * 0.02)]),
      createTransform([0, -0.44, 0]),
      createTransform([0, -0.4 + Math.max(0, swing) * 0.08, stride * swing]),
      createTransform([0, 0.3, 0]), createTransform([0, 0.3, 0]),
    ];
  });
  return new AnimationClip(name, duration, syntheticRig.bones.map((_, bone) =>
    new AnimationTrack(bone, frames.map((frame, index) =>
      new Keyframe(index * duration / count, frame[bone])))));
}

export const syntheticClips = Object.freeze(Object.fromEntries(definitions.map((definition) =>
  [definition.name, makeClip(definition)])));
export const syntheticDefinitions = Object.freeze(Object.fromEntries(definitions.map((definition) =>
  [definition.name, Object.freeze(definition)])));

export function buildSyntheticPoseDatabase() {
  const samples = [];
  const sampleTimes = [0.2, 0.4, 0.6, 0.8, 1];
  for (const definition of definitions) {
    const clip = syntheticClips[definition.name];
    const duration = clip.durationSeconds;
    const radians = definition.direction * Math.PI / 180;
    const velocity = [Math.sin(radians) * definition.speed, 0,
      Math.cos(radians) * definition.speed];
    const trajectory = sampleTimes.map((time) => ({ time,
      position: velocity.map((component) => component * time),
      velocity: [...velocity], facing: definition.direction }));
    const history = new PoseHistory(32);
    const dt = duration / 16;
    for (let index = 0; index < 16; index++) {
      const time = index * dt;
      const phase = time / duration;
      const pose = sampleAnimationClip(clip, syntheticRig, time);
      const contacts = { left: phase < 0.25 || phase >= 0.75,
        right: phase >= 0.25 && phase < 0.75 };
      const sample = history.push(pose, { tick: index + 1, rootVelocity: velocity,
        trajectory, contacts, clipName: clip.name, timeSeconds: time, dt });
      if (index % 2 === 1) samples.push({ id: sample.id, clipName: clip.name, timeSeconds: time,
        pose, features: sample.features, metadata: { durationSeconds: duration,
          tag: definition.tag, gait: definition.gait ?? 'idle', markers: phaseMarkers,
          referenceSpeed: definition.speed, contacts } });
    }
  }
  return new PoseDatabase(samples);
}
