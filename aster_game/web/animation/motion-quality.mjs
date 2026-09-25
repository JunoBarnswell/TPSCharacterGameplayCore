import { worldTransforms } from './pose.mjs';

const length = (vector) => Math.hypot(...vector);
const difference = (a, b) => a.map((value, index) => value - b[index]);

/** Deterministic presentation trace; all distances are metres and time is seconds. */
export class MotionQualityTrace {
  constructor(limit = 600) {
    this.limit = limit;
    this.samples = [];
    this.clipSwitchCount = 0;
    this.maxFootPlantDrift = 0;
    this.maxOrientationAngularVelocity = 0;
    this.maxBoneVelocityJump = 0;
    this.maxRemoteYawVelocity = 0;
    this.remoteActionTimingOffsetTicks = [];
    this.predictionErrors = [];
    this.hardResyncCount = 0;
  }

  push(frame, output, dt) {
    if (!(dt > 0) || !Number.isFinite(dt) || !output.pose) {
      throw new TypeError('motion quality requires a final pose and positive timestep');
    }
    const previous = this.samples.at(-1);
    const worlds = worldTransforms(output.pose);
    const bones = Object.fromEntries(['pelvis', 'left_foot', 'right_foot'].map((name) =>
      [name, worlds[output.pose.skeleton.indexByName.get(name)].translation]));
    const velocity = previous ? difference(frame.position, previous.position).map((v) => v / dt) : [0, 0, 0];
    const acceleration = previous ? difference(velocity, previous.velocity).map((v) => v / dt) : [0, 0, 0];
    const jerk = previous ? difference(acceleration, previous.acceleration).map((v) => v / dt) : [0, 0, 0];
    if (previous && output.selectedClip !== previous.selectedClip) this.clipSwitchCount++;
    const beforePrevious = this.samples.at(-2);
    if (previous && beforePrevious && output.selectedClip !== previous.selectedClip) {
      for (const name of ['pelvis', 'left_foot', 'right_foot']) {
        const velocityBefore = difference(previous.bones[name], beforePrevious.bones[name]);
        const velocityAfter = difference(bones[name], previous.bones[name]);
        this.maxBoneVelocityJump = Math.max(this.maxBoneVelocityJump,
          length(difference(velocityAfter, velocityBefore)) / dt);
      }
    }
    const footDrift = {};
    for (const side of ['left', 'right']) {
      const planted = output.footPlantStates[side].state === 'planted';
      footDrift[side] = previous && planted && previous.footPlantStates[side].state === 'planted'
        ? length(difference(bones[`${side}_foot`], previous.bones[`${side}_foot`])) : 0;
      this.maxFootPlantDrift = Math.max(this.maxFootPlantDrift, footDrift[side]);
    }
    const orientationRate = previous
      ? Math.abs(output.warpValues.orientation - previous.warpAngle) / dt : 0;
    this.maxOrientationAngularVelocity = Math.max(this.maxOrientationAngularVelocity, orientationRate);
    const sample = { tick: frame.tick, position: [...frame.position], velocity,
      acceleration, jerk, pelvis: bones.pelvis, bones, footDrift,
      selectedClip: output.selectedClip, playbackTimeSeconds: output.playbackTimeSeconds,
      footPlantStates: output.footPlantStates, warpAngle: output.warpValues.orientation,
      candidateCount: output.match.candidateCount,
      rootSpeed: length(velocity), accelerationMagnitude: length(acceleration),
      jerkMagnitude: length(jerk), orientationRate };
    this.samples.push(sample);
    if (this.samples.length > this.limit) this.samples.shift();
    return sample;
  }

  observeRemote(previous, current, dt) {
    if (previous && current && dt > 0) {
      const delta = ((current.character_yaw - previous.character_yaw + 540) % 360) - 180;
      this.maxRemoteYawVelocity = Math.max(this.maxRemoteYawVelocity, Math.abs(delta) / dt);
    }
    for (const event of current?.action_events ?? []) {
      this.remoteActionTimingOffsetTicks.push(current.render_tick - event.start_tick);
    }
  }

  observePrediction(error, hardResync = false) {
    if (!Number.isFinite(error) || error < 0) throw new RangeError('prediction error must be finite');
    this.predictionErrors.push(error);
    if (hardResync) this.hardResyncCount++;
  }

  summary() {
    const errors = [...this.predictionErrors].sort((a, b) => a - b);
    return { samples: this.samples.length, clipSwitchCount: this.clipSwitchCount,
      maxFootPlantDrift: this.maxFootPlantDrift,
      maxOrientationAngularVelocity: this.maxOrientationAngularVelocity,
      maxBoneVelocityJump: this.maxBoneVelocityJump,
      maxRemoteYawVelocity: this.maxRemoteYawVelocity,
      maxRemoteActionTimingOffsetTicks: Math.max(0,
        ...this.remoteActionTimingOffsetTicks.map(Math.abs)),
      predictionCorrectionP95: errors.length ? errors[Math.ceil(errors.length * 0.95) - 1] : 0,
      predictionCorrectionMax: errors.at(-1) ?? 0,
      hardResyncCount: this.hardResyncCount,
      candidateSelectionCount: this.samples.reduce((sum, sample) => sum + sample.candidateCount, 0),
      maxRootAcceleration: Math.max(0, ...this.samples.map((s) => s.accelerationMagnitude)),
      maxRootJerk: Math.max(0, ...this.samples.map((s) => s.jerkMagnitude)) };
  }
}
