import { createPose, createTransform, Pose, worldTransforms } from './pose.mjs';
import { PoseInertializer } from './pose-inertializer.mjs';
import { OrientationWarpState, warpPoseOrientation } from './orientation-warp.mjs';
import { evaluateAimOffsetPose } from './aim-offset.mjs';
import { evaluateAimOffset } from './aim-offset.mjs';
import { evaluateBlendSpace } from './blend-space.mjs';
import { AnimationLayerChannel, BoneMask, blendAnimationLayers } from './layers.mjs';
import { solveFootIK } from './foot-ik.mjs';
import { extractRootMotionDelta } from './root-motion.mjs';
import { MotionWarpTarget, warpRootMotionDelta } from './motion-warp.mjs';
import { PoseHistory } from './motion-matching/pose-history.mjs';
import { MotionMatcher } from './motion-matching/motion-matcher.mjs';
import { PoseSearch } from './motion-matching/pose-search.mjs';
import { syntheticRig, syntheticClips, syntheticDefinitions,
  buildSyntheticPoseDatabase } from './synthetic-library.mjs';

function rotation(axis, degrees) {
  const half = degrees * Math.PI / 360;
  return axis.map((value) => value * Math.sin(half)).concat(Math.cos(half));
}

function additivePose(skeleton, boneRotations) {
  const transforms = skeleton.bones.map(() => createTransform());
  for (const [name, rotationValue] of Object.entries(boneRotations)) {
    transforms[skeleton.indexByName.get(name)] = createTransform([0, 0, 0], rotationValue);
  }
  return createPose(skeleton, transforms);
}

function aimLibrary(skeleton) {
  const result = {};
  for (const [name, yaw, pitch] of [
    ['center', 0, 0], ['left', -90, 0], ['right', 90, 0],
    ['up', 0, 60], ['down', 0, -60], ['up_left', -90, 60],
    ['up_right', 90, 60], ['down_left', -90, -60], ['down_right', 90, -60],
  ]) {
    // Simple synthetic aim samples; production assets can replace these poses.
    result[name] = additivePose(skeleton, {
      chest: rotation([0, 1, 0], yaw * 0.35),
      head: rotation([1, 0, 0], pitch * 0.55),
    });
  }
  return result;
}

export class FootPlantState {
  constructor() {
    this.state = 'free'; this.target = null; this.weight = 0;
    this.acquireTime = 0; this.lostTime = 0;
  }
  update({ contact, support, grounded, position, speed = 0 }, dt) {
    const valid = grounded && support && contact >= 0.6 && speed < 1.5;
    if (valid) { this.acquireTime += dt; this.lostTime = 0; }
    else { this.acquireTime = 0; this.lostTime += dt; }
    if (this.state === 'free' && valid) this.state = 'candidate';
    if (this.state === 'candidate' && this.acquireTime >= 0.025) {
      this.state = 'planting'; this.target = [...position];
    }
    if (['planting', 'planted'].includes(this.state) &&
        (this.lostTime > 0.06 || (this.target && Math.hypot(...position.map((v, i) => v - this.target[i])) > 0.55))) {
      this.state = 'releasing';
    }
    if (this.state === 'candidate' && !valid) this.state = 'free';
    const step = dt / 0.065;
    this.weight = Math.max(0, Math.min(1, this.weight +
      (['planting', 'planted'].includes(this.state) ? step : -step)));
    if (this.state === 'planting' && this.weight >= 1) this.state = 'planted';
    if (this.state === 'releasing' && this.weight <= 0) {
      this.state = 'free'; this.target = null;
    }
    return { state: this.state, weight: this.weight, target: this.target };
  }
  reset() { this.state = 'free'; this.target = null; this.weight = 0; this.acquireTime = 0; this.lostTime = 0; }
}

function setRoot(pose, position, yaw) {
  const transforms = [...pose.localTransforms];
  transforms[0] = createTransform(position, rotation([0, 1, 0], yaw));
  return new Pose(pose.skeleton, transforms);
}

export function warpPoseStride(pose, scale) {
  if (Math.abs(scale - 1) < 0.001) return pose;
  const transforms = [...pose.localTransforms];
  for (const name of ['left_foot', 'right_foot']) {
    const index = pose.skeleton.indexByName.get(name);
    if (index === undefined) continue;
    const foot = transforms[index];
    transforms[index] = createTransform([foot.translation[0], foot.translation[1],
      foot.translation[2] * scale], foot.rotation, foot.scale);
  }
  return new Pose(pose.skeleton, transforms);
}

export class CharacterAnimationRuntime {
  constructor({ skeleton = syntheticRig, clips = syntheticClips,
    database = buildSyntheticPoseDatabase(), groundProbe = null } = {}) {
    this.skeleton = skeleton; this.clips = clips; this.groundProbe = groundProbe;
    this.matcher = new MotionMatcher(new PoseSearch(database), { candidateLimit: 8, clips });
    this.inertializer = new PoseInertializer(0.12);
    this.orientation = new OrientationWarpState();
    this.plants = { left: new FootPlantState(), right: new FootPlantState() };
    this.history = new PoseHistory(180);
    this.aimPoses = aimLibrary(skeleton);
    this.aimMask = BoneMask.fromNames(skeleton, ['chest', 'head']);
    this.upperMask = BoneMask.fromNames(skeleton, ['spine', 'chest', 'head']);
    this.previousPose = null; this.previousPreviousPose = null; this.previousTarget = null;
    this.previousSignature = null; this.transitionTime = null;
    this.switchCount = 0;
    this.pelvisOffset = 0; this.footNormals = { left: [0, 1, 0], right: [0, 1, 0] };
    this.footCorrections = { left: [0, 0, 0], right: [0, 0, 0] };
    this.previousFootWorld = { left: null, right: null };
  }

  update(frame, { trajectory = [], dt = 1 / 60, maxSpeed = 6.5,
    motionWarpTarget = null, tickRate = 60,
    groundProbe = this.groundProbe } = {}) {
    if (!(dt > 0) || !Number.isFinite(dt)) throw new RangeError('runtime timestep must be positive');
    const phase = frame.locomotionPhase;
    const tag = ['jump_start', 'rising', 'apex', 'falling'].includes(phase) ? 'airborne' :
      ['soft_land', 'normal_land', 'heavy_land'].includes(phase) ? 'landing' :
      ['pivot', 'turn_in_place'].includes(phase) ? 'turn' : 'grounded';
    const clipNames = tag === 'airborne' ? [phase === 'apex' ? 'apex' : phase] :
      tag === 'landing' ? [phase] : tag === 'turn' ?
        [phase === 'pivot' ? 'pivot_reverse' : frame.turnDirection === 'left'
          ? 'turn_left_90' : 'turn_right_90'] : null;
    const queryPose = this.previousPose ?? setRoot(
      this.matcher.samplers.get('idle').sample(0), frame.position, frame.characterYaw);
    const times = [0.2, 0.4, 0.6, 0.8, 1];
    const searchTrajectory = trajectory.length === times.length ? trajectory : times.map((time) => ({
      time,
      position: frame.position.map((value, axis) => value + frame.velocity[axis] * time),
      velocity: [...frame.velocity], facing: frame.characterYaw,
    }));
    const previousSample = this.history.latest(1)[0] ?? null;
    const contacts = { left: frame.grounded && frame.gaitPhase < 0.5,
      right: frame.grounded && frame.gaitPhase >= 0.5 };
    const playbackRate = this.matcher.currentCandidate && tag === 'grounded'
      ? Math.max(0.85, Math.min(1.15, frame.horizontalSpeed /
        Math.max(0.35, syntheticDefinitions[this.matcher.currentCandidate.candidate.clipName]?.speed ?? 1))) : 1;
    const match = this.matcher.update({ pose: queryPose, rootVelocity: frame.velocity,
      trajectory: searchTrajectory, previousSample, contacts, dt, tag,
      playbackRate, clipNames, forceSwitch: tag !== 'grounded' });
    let target = match.selectedPose;
    const definition = syntheticDefinitions[match.selectedClip];
    const referenceSpeed = definition?.speed ?? 0;
    const strideScale = tag === 'grounded' && frame.horizontalSpeed > 0.35 && referenceSpeed > 0.35
      ? Math.max(0.72, Math.min(1.35, frame.horizontalSpeed / (referenceSpeed * playbackRate))) : 1;
    target = warpPoseStride(target, strideScale);
    target = setRoot(target, frame.position, frame.characterYaw);
    const signature = `${match.selectedClip}|${tag}`;
    if (this.previousPose && signature !== this.previousSignature) {
      const clip = this.clips[match.selectedClip];
      const previousTime = (match.playbackTimeSeconds - dt + clip.durationSeconds) % clip.durationSeconds;
      const incomingPrevious = setRoot(this.matcher.samplers.get(match.selectedClip).sample(previousTime),
        frame.position, frame.characterYaw);
      this.inertializer.begin(this.previousPose, target, {
        previousOutput: this.previousPreviousPose,
        previousTarget: incomingPrevious, dt,
      });
      this.transitionTime = 0;
      this.switchCount++;
    }
    this.previousSignature = signature;
    if (this.transitionTime !== null) {
      target = this.inertializer.sample(target, this.transitionTime);
      this.transitionTime += dt;
      if (this.transitionTime > 0.6) { this.inertializer.clear(); this.transitionTime = null; }
    }
    const aimed = evaluateAimOffsetPose(target, this.aimPoses,
      frame.aimYaw, frame.aimPitch, 1, this.aimMask);
    const layers = frame.actionLayers.map(({ channel, state, normalized_progress: progress,
      end_tick: endTick }) => {
      const degrees = channel === 'additive_reaction' ? 14 * (frame.hitStrength || 1) :
        channel === 'life_override' ? 60 : channel === 'upper_body_action' ? -18 : 25;
      const pose = additivePose(this.skeleton, {
        spine: rotation([1, 0, 0], degrees * 0.4),
        chest: rotation([1, 0, 0], degrees),
        head: rotation([0, 0, 1], channel === 'life_override' ? 28 : 4),
      });
      const overridePose = channel === 'life_override'
        ? blendAnimationLayers(aimed, [{ name: 'death-source',
          channel: AnimationLayerChannel.LIFE_OVERRIDE, pose, mode: 'additive', weight: 1 }])
        : pose;
      const normalizedWeight = channel === 'upper_body_action' && endTick !== null &&
        Number.isFinite(progress)
        ? Math.max(0, Math.min(1, progress / 0.15, (1 - progress) / 0.2)) : 1;
      return { name: `${channel}:${state}`, channel: AnimationLayerChannel[channel.toUpperCase()],
        pose: overridePose, mode: channel === 'life_override' ? 'override' : 'additive',
        weight: normalizedWeight,
        boneMask: channel === 'life_override' ? BoneMask.fullBody(this.skeleton) : this.upperMask };
    });
    let layered = blendAnimationLayers(aimed, layers);
    const warpAngle = this.orientation.update(frame.worldMovementDirection, frame.characterYaw, dt,
      tag === 'turn' || tag === 'airborne');
    layered = warpPoseOrientation(layered, frame.worldMovementDirection, frame.characterYaw,
      { root: 0.1, pelvis: 0.15, spine: 0.2, left_thigh: 0.075,
        right_thigh: 0.075, chest: 0.25, head: 0.15 }, 90, warpAngle).pose;
    const probes = {};
    const worlds = worldTransforms(layered);
    const clipPhase = (match.playbackTimeSeconds / this.clips[match.selectedClip].durationSeconds) % 1;
    const clipContacts = { left: clipPhase < 0.25 || clipPhase >= 0.75,
      right: clipPhase >= 0.25 && clipPhase < 0.75 };
    for (const side of ['left', 'right']) {
      const foot = worlds[this.skeleton.indexByName.get(`${side}_foot`)].translation;
      const result = groundProbe?.(foot, side) ?? { grounded: false };
      const footSpeed = this.previousFootWorld[side]
        ? Math.hypot(...foot.map((value, axis) =>
          (value - this.previousFootWorld[side][axis]) / dt)) : 0;
      this.previousFootWorld[side] = foot;
      const plant = this.plants[side].update({ contact: Number(tag === 'grounded' && clipContacts[side]),
        support: Boolean(result.grounded), grounded: frame.grounded,
        position: result.position ?? foot, speed: footSpeed }, dt);
      const grace = Boolean(frame.grounded && plant.target && plant.weight > 0 &&
        this.plants[side].lostTime <= 0.06);
      probes[side] = { grounded: frame.grounded && (Boolean(result.grounded) || grace),
        position: result.position ?? plant.target ?? foot,
        normal: result.normal ?? this.footNormals[side],
        lockedPosition: plant.target && plant.weight > 0.5 ? plant.target : null };
    }
    const footIK = solveFootIK(layered, probes, { previousPelvisOffset: this.pelvisOffset,
      previousNormals: this.footNormals, previousFootCorrections: this.footCorrections, dt });
    this.pelvisOffset = footIK.pelvisOffset;
    for (const side of ['left', 'right']) {
      if (footIK.footNormals[side]) this.footNormals[side] = footIK.footNormals[side];
      this.footCorrections[side] = Array.isArray(footIK.footOffsets[side])
        ? footIK.footOffsets[side] : [0, 0, 0];
    }
    const finalPose = footIK.pose;
    let rootMotionPreview = this.previousTarget
      ? extractRootMotionDelta(this.previousTarget, match.selectedPose)
      : { translation: [0, 0, 0], yawDelta: 0 };
    const vault = frame.actionChannels.full_body_override;
    if (motionWarpTarget instanceof MotionWarpTarget && vault.active &&
        vault.state === 'synthetic_vault') {
      rootMotionPreview = warpRootMotionDelta(rootMotionPreview,
        { position: frame.position, character_yaw: frame.characterYaw },
        [0, 0, 0], motionWarpTarget, 1, 0,
        Math.max(0, (frame.tick - vault.start_tick) / tickRate));
    }
    this.previousTarget = match.selectedPose;
    this.previousPreviousPose = this.previousPose;
    this.previousPose = finalPose;
    this.history.push(finalPose, { tick: this.history.samples.length ?
      this.history.samples.at(-1).tick + 1 : 1, rootVelocity: frame.velocity,
      trajectory: searchTrajectory, contacts, clipName: match.selectedClip,
      timeSeconds: match.playbackTimeSeconds, dt });
    return { pose: finalPose, selectedClip: match.selectedClip,
      semantic: { locomotion: evaluateBlendSpace(frame, maxSpeed),
        aimOffset: evaluateAimOffset(frame.aimYaw, frame.aimPitch),
        actionLayers: frame.actionLayers, orientationWarp: warpAngle },
      playbackTimeSeconds: match.playbackTimeSeconds, match, rootMotionPreview,
      activeLayers: layers.map(({ name }) => name), inertializationState: this.transitionTime,
      footPlantStates: Object.fromEntries(Object.entries(this.plants).map(([side, plant]) =>
        [side, { state: plant.state, weight: plant.weight }])),
      footIK, warpValues: { orientation: warpAngle, stride: strideScale },
      diagnostics: { switchCount: this.switchCount, historySamples: this.history.samples.length } };
  }

  reset() {
    this.matcher.reset(); this.inertializer.clear(); this.orientation.reset();
    this.plants.left.reset(); this.plants.right.reset(); this.history.clear();
    this.previousPose = null; this.previousPreviousPose = null; this.previousTarget = null;
    this.previousSignature = null; this.transitionTime = null; this.switchCount = 0;
    this.pelvisOffset = 0; this.footNormals = { left: [0, 1, 0], right: [0, 1, 0] };
    this.footCorrections = { left: [0, 0, 0], right: [0, 0, 0] };
    this.previousFootWorld = { left: null, right: null };
  }
}
