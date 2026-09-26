import test from 'node:test';
import assert from 'node:assert/strict';
import { CharacterAnimationRuntime } from '../aster_game/web/animation/character-animation-runtime.mjs';
import { syntheticRig, syntheticClips, buildSyntheticPoseDatabase } from '../aster_game/web/animation/synthetic-library.mjs';
import { sampleAnimationClip } from '../aster_game/web/animation/clip.mjs';
import { createMotionFrame } from '../aster_game/web/motion/motion-frame.mjs';
import { PoseHistory } from '../aster_game/web/animation/motion-matching/pose-history.mjs';
import { PoseDatabase } from '../aster_game/web/animation/motion-matching/pose-database.mjs';
import { PoseSearch } from '../aster_game/web/animation/motion-matching/pose-search.mjs';
import { extractPoseFeatures } from '../aster_game/web/animation/motion-matching/pose-features.mjs';
import { createPose, createTransform, worldTransforms } from '../aster_game/web/animation/pose.mjs';
import { solveTwoBoneLeg } from '../aster_game/web/animation/foot-ik.mjs';
import { OrientationWarpState } from '../aster_game/web/animation/orientation-warp.mjs';
import { MotionWarpTarget, warpRootMotionDelta } from '../aster_game/web/animation/motion-warp.mjs';
import { InputEdgeBuffer, RemoteSnapshotBuffer, FixedStepScheduler } from '../aster_game/web/motion/network-motion.mjs';
import { evaluateBlendSpace } from '../aster_game/web/animation/blend-space.mjs';
import { synchronizeMarkerTime } from '../aster_game/web/animation/motion-matching/motion-matcher.mjs';

const near = (a, b, epsilon = 1e-5) => assert.ok(Math.abs(a - b) < epsilon, `${a} != ${b}`);
function frame(tick, overrides = {}) {
  return createMotionFrame({ position: [0, 0, tick * 4.5 / 60], velocity: [0, 0, 4.5],
    horizontal_speed: 4.5, character_yaw: 0, grounded: true,
    locomotion_phase: 'run', gait_phase: tick / 60 % 1, ...overrides }, tick, 1 / 60);
}

test('empty trajectory normalization remains finite and searchable', () => {
  const pose = sampleAnimationClip(syntheticClips.idle, syntheticRig, 0);
  const sample = new PoseHistory(1).push(pose, { tick: 1, rootVelocity: [0, 0, 0] });
  const search = new PoseSearch(new PoseDatabase([{ id: sample.id, clipName: 'idle',
    timeSeconds: 0, pose, features: sample.features }]));
  near(search.normalizationScales.trajectory, 0.5);
  assert.ok(Object.values(search.normalizationScales).every(Number.isFinite));
  assert.ok(Object.values(search.dimensionScales).flat().every(Number.isFinite));
  near(search.search(sample.features).results[0].cost, 0);
});

test('clip-backed runtime advances sample time and changes final skeleton for aim and actions', () => {
  const runtime = new CharacterAnimationRuntime();
  const base = runtime.update(frame(0));
  const next = runtime.update(frame(1));
  near(next.playbackTimeSeconds, (base.playbackTimeSeconds + 1 / 60) % 1);
  assert.equal(next.pose.skeleton, syntheticRig);
  assert.equal(next.selectedClip, 'run_forward');
  const clipSample = sampleAnimationClip(syntheticClips[next.selectedClip], syntheticRig, next.playbackTimeSeconds);
  assert.deepEqual(next.match.selectedPose.localTransforms[5], clipSample.localTransforms[5]);
  const withAim = new CharacterAnimationRuntime().update(frame(0, { aim_yaw: 60, aim_pitch: 40 }));
  assert.notDeepEqual(withAim.pose.localTransforms[9].rotation, base.pose.localTransforms[9].rotation);
  const shoot = new CharacterAnimationRuntime().update(frame(0, { action_channels: {
    upper_body_action: { active: true, state: 'shoot', sequence: 1 },
  } }));
  assert.ok(shoot.activeLayers.some((name) => name.includes('shoot')));
  assert.notDeepEqual(shoot.pose.localTransforms[9].rotation, base.pose.localTransforms[9].rotation);
  const death = new CharacterAnimationRuntime().update(frame(0, { action_channels: {
    life_override: { active: true, state: 'death', sequence: 1 },
  } }));
  assert.notDeepEqual(death.pose.localTransforms[10].rotation, base.pose.localTransforms[10].rotation);
});

test('database has temporal foot velocities and distinct phase tags', () => {
  const database = buildSyntheticPoseDatabase();
  const run = database.candidates.filter((sample) => sample.clipName === 'run_forward');
  assert.ok(run.some((sample) => sample.features.leftFootVelocity[2] > 0));
  assert.ok(run.some((sample) => sample.features.leftFootVelocity[2] < 0));
  assert.ok(database.normalization.every(({ mean, scale }) => Number.isFinite(mean) && scale >= 0.25));
  for (const tag of ['grounded', 'turn', 'landing', 'airborne']) {
    assert.ok(database.candidates.some((sample) => sample.metadata.tag === tag));
  }
  const runtime = new CharacterAnimationRuntime({ database });
  runtime.update(frame(0));
  const falling = runtime.update(frame(1, { grounded: false, locomotion_phase: 'falling' }));
  assert.equal(database.candidates.find((c) => c.clipName === falling.selectedClip).metadata.tag, 'airborne');
});

test('canonical local features do not change when the complete world scene rotates', () => {
  const pose = sampleAnimationClip(syntheticClips.run_forward, syntheticRig, 0.125);
  const trajectory = [0.2, 0.4].map((time) => ({ position: [1, 0, 4.5 * time],
    velocity: [0, 0, 4.5], facing: 0 }));
  const baseline = extractPoseFeatures({ pose, rootVelocity: [0, 0, 4.5], trajectory });
  for (const degrees of [0, 45, 90, 180]) {
    const rad = degrees * Math.PI / 180;
    const rotate = ([x, y, z]) => [x * Math.cos(rad) + z * Math.sin(rad), y,
      -x * Math.sin(rad) + z * Math.cos(rad)];
    const local = [...pose.localTransforms];
    local[0] = createTransform([0, 0, 0], [0, Math.sin(rad / 2), 0, Math.cos(rad / 2)]);
    const turned = createPose(syntheticRig, local);
    const features = extractPoseFeatures({ pose: turned, rootVelocity: rotate([0, 0, 4.5]),
      trajectory: trajectory.map((sample) => ({ ...sample,
        position: rotate(sample.position), velocity: rotate(sample.velocity), facing: degrees })) });
    baseline.vector.forEach((value, index) => near(value, features.vector[index], 1e-4));
  }
});

test('directional sprint contains each full-speed exact sample', () => {
  for (const degrees of [0, 45, 90, 135, 180, -135, -90, -45]) {
    const name = new Map([[0, 'sprint'], [45, 'sprint_forward_right'], [90, 'sprint_right'],
      [135, 'sprint_backward_right'], [180, 'sprint_backward'], [-135, 'sprint_backward_left'],
      [-90, 'sprint_left'], [-45, 'sprint_forward_left']]).get(degrees);
    near(evaluateBlendSpace({ movementDirection: degrees, horizontalSpeed: 6.5 }, 6.5)[name], 1);
  }
});

test('marker synchronization preserves plant identity across locomotion switches', () => {
  const db = buildSyntheticPoseDatabase();
  const names = ['walk_forward', 'run_forward', 'sprint', 'run_forward_right', 'run_right'];
  for (let i = 0; i < names.length - 1; i++) {
    const a = db.candidates.find((sample) => sample.clipName === names[i]);
    const b = db.candidates.find((sample) => sample.clipName === names[i + 1]);
    for (const phase of [0.01, 0.24, 0.49, 0.51, 0.75, 0.98]) {
      near(synchronizeMarkerTime(a, phase * a.metadata.durationSeconds, b) /
        b.metadata.durationSeconds, phase);
    }
  }
});

test('two bone leg places reachable foot by rotating chain without translating foot child', () => {
  const source = sampleAnimationClip(syntheticClips.idle, syntheticRig, 0);
  const target = [-0.2, -0.78, 0.15];
  const result = solveTwoBoneLeg(source, 'left', target);
  assert.ok(result.reachable);
  assert.ok(result.error < 0.02, `IK residual ${result.error}`);
  assert.deepEqual(result.pose.localTransforms[5].translation, source.localTransforms[5].translation);
});

test('rear seam orientation and remote yaw derivatives remain bounded', () => {
  const state = new OrientationWarpState({ maxAngularVelocity: 360 });
  let last = null;
  for (const heading of [170, 175, 179, -179, -175, -170]) {
    const result = state.update(heading, 0, 1 / 60);
    if (last !== null) assert.ok(Math.abs(result - last) <= 6.000001);
    last = result;
  }
  const remote = new RemoteSnapshotBuffer(8, 10, { maxVisualYawRate: 720 });
  remote.push({ tick: 0, position: [0, 0, 0], velocity: [0, 0, 0], character_yaw: 0,
    yaw_rate: 0, gait_phase: 0.98, locomotion_phase: 'run' });
  remote.push({ tick: 3, position: [0, 0, 0], velocity: [0, 0, 0], character_yaw: 90,
    yaw_rate: 0, gait_phase: 0.02, locomotion_phase: 'sprint' });
  let previous = remote.sample(0, 60).character_yaw;
  for (let tick = 0.025; tick <= 3; tick += 0.025) {
    const sample = remote.sample(Math.min(tick, 3), 60);
    const delta = ((sample.character_yaw - previous + 540) % 360) - 180;
    assert.ok(Math.abs(delta * 60 / 0.025) <= 720.001);
    if (tick < 3) assert.equal(sample.locomotion_phase, 'run');
    previous = sample.character_yaw;
  }
  near(remote.sample(1.5, 60).gait_phase, 0);
  near(remote.sample(3, 60).character_yaw, 36);
});

test('zero authored warp is zero; short jump press survives fixed ticks and stalls report loss', () => {
  const target = new MotionWarpTarget('test', [1, 0, 0], 180);
  const warped = warpRootMotionDelta({ translation: [0, 0, 0], yawDelta: 0 },
    { position: [0, 0, 0], character_yaw: 0 }, [0, 0, 0], target, 1, 0);
  assert.deepEqual(warped.translation, [0, 0, 0]);
  near(warped.yawDelta, 0);
  const edges = new InputEdgeBuffer();
  edges.keyDown(); edges.keyUp();
  assert.deepEqual(edges.consume(), { jump: true, jump_pressed: true, jump_released: true });
  assert.equal(edges.consume().jump, false);
  const scheduler = new FixedStepScheduler(60, 4);
  scheduler.reset(0); scheduler.advance(200);
  assert.ok(scheduler.dropped_time_ms > 100);
  assert.ok(scheduler.dropped_step_count >= 7);
  assert.equal(scheduler.max_catchup_hit_count, 1);
});

test('remote actions begin on the delayed render tick and initial active actions enter at progress', async () => {
  const { RemoteRenderMotionStateSampler } = await import('../aster_game/web/motion/network-motion.mjs');
  const remote = new RemoteRenderMotionStateSampler();
  const sample = (tick, shoot) => ({ tick, position: [tick / 10, 0, 0],
    velocity: [2, 0, 0], character_yaw: 0, life_state: 'alive',
    action_channels: { upper_body_action: { active: shoot, state: shoot ? 'shoot' : 'none',
      sequence: shoot ? 1 : 0, event_id: shoot ? 'shot-1' : '', start_tick: shoot ? 12 : 0,
      end_tick: shoot ? 21 : null } } });
  remote.push(sample(9, false));
  remote.push(sample(12, true)); // Packet arrives before the visual timeline reaches tick 12.
  assert.equal(remote.sample(11, 60).action_channels.upper_body_action.active, false);
  const atShot = remote.sample(12, 60);
  assert.equal(atShot.action_channels.upper_body_action.active, true);
  assert.equal(atShot.action_events.length, 1);
  assert.equal(remote.sample(13, 60).action_events.length, 0);
  remote.push(sample(15, true));
  near(remote.sample(15, 60).action_channels.upper_body_action.normalized_progress, 1 / 3);
  const newcomer = new RemoteRenderMotionStateSampler();
  newcomer.push(sample(15, true));
  const active = newcomer.sample(15, 60);
  near(active.action_channels.upper_body_action.normalized_progress, 1 / 3);
  assert.equal(active.action_events.length, 0);
  assert.equal(newcomer.sample(22, 60).action_channels.upper_body_action.active, false);
});

test('stride warp reduces planted foot drift at 3 and 6 m/s around a 4.5 m/s authored run', async () => {
  const { warpPoseStride } = await import('../aster_game/web/animation/character-animation-runtime.mjs');
  const footWorldZ = (time, speed, strideScale, playbackRate) => {
    const clipPose = sampleAnimationClip(syntheticClips.run_forward, syntheticRig, time * playbackRate);
    const adjusted = warpPoseStride(clipPose, strideScale);
    return worldTransforms(adjusted)[syntheticRig.indexByName.get('left_foot')].translation[2] + speed * time;
  };
  for (const speed of [3, 4.5, 6]) {
    const playbackRate = Math.max(0.85, Math.min(1.15, speed / 4.5));
    const scale = Math.max(0.72, Math.min(1.35, speed / (4.5 * playbackRate)));
    const baseline = Math.abs(footWorldZ(0.06, speed, 1, playbackRate) - footWorldZ(0, speed, 1, playbackRate));
    const warped = Math.abs(footWorldZ(0.06, speed, scale, playbackRate) -
      footWorldZ(0, speed, scale, playbackRate));
    if (speed !== 4.5) assert.ok(warped < baseline * 0.65,
      `${speed}m/s slide ${baseline.toFixed(4)} -> ${warped.toFixed(4)}`);
    else near(warped, baseline);
  }
});

test('noisy foot probes retain planting through one missed tick and smooth pelvis correction', async () => {
  const { FootPlantState } = await import('../aster_game/web/animation/character-animation-runtime.mjs');
  const plant = new FootPlantState();
  for (let i = 0; i < 5; i++) plant.update({ contact: 1, support: true, grounded: true,
    position: [0, -0.9, 0] }, 1 / 60);
  assert.equal(plant.state, 'planted');
  const missed = plant.update({ contact: 1, support: false, grounded: true,
    position: [0, -0.9, 0] }, 1 / 60);
  assert.equal(missed.state, 'planted');
  plant.update({ contact: 1, support: true, grounded: true,
    position: [0, -0.89, 0] }, 1 / 60);
  assert.equal(plant.state, 'planted');
  const { solveFootIK } = await import('../aster_game/web/animation/foot-ik.mjs');
  const pose = sampleAnimationClip(syntheticClips.idle, syntheticRig, 0);
  const result = solveFootIK(pose, { left: { grounded: true, position: [-0.2, -0.5, 0] },
    right: { grounded: true, position: [0.2, -0.5, 0] } },
  { previousPelvisOffset: 0, dt: 1 / 60 });
  assert.ok(Math.abs(result.pelvisOffset) <= 1.2 / 60 + 1e-9);
});

test('deterministic motion quality trace bounds orientation and clip switching', async () => {
  const { MotionQualityTrace } = await import('../aster_game/web/animation/motion-quality.mjs');
  const runtime = new CharacterAnimationRuntime();
  const trace = new MotionQualityTrace();
  for (let tick = 0; tick < 60; tick++) {
    const speed = tick < 15 ? 0 : tick < 30 ? 2 : tick < 45 ? 4.5 : 6.5;
    const yaw = tick < 45 ? 0 : 45;
    const state = frame(tick, { position: [0, 0, tick * speed / 60],
      velocity: [0, 0, speed], horizontal_speed: speed,
      character_yaw: yaw, actual_gait: speed < 1 ? 'idle' : speed < 3 ? 'walk' :
        speed < 6 ? 'run' : 'sprint' });
    const output = runtime.update(state);
    trace.push(state, output, 1 / 60);
  }
  const summary = trace.summary();
  assert.equal(summary.samples, 60);
  assert.ok(summary.clipSwitchCount >= 2 && summary.clipSwitchCount < 12);
  assert.ok(summary.maxOrientationAngularVelocity <= 540.0001);
  assert.ok(trace.samples.every((sample) => Number.isFinite(sample.jerkMagnitude) &&
    Number.isFinite(sample.pelvis[0])));
});

test('inertial transitions preserve C0 and approximate linear and angular velocity across interruption', async () => {
  const { PoseInertializer } = await import('../aster_game/web/animation/pose-inertializer.mjs');
  const skeleton = syntheticRig;
  const dt = 1 / 60;
  const poseAt = (x, angle) => {
    const transforms = skeleton.bones.map((bone) => bone.bindLocal);
    transforms[9] = createTransform([x, 0.3, 0],
      [0, Math.sin(angle / 2), 0, Math.cos(angle / 2)]);
    return createPose(skeleton, transforms);
  };
  const inertial = new PoseInertializer(0.12);
  inertial.begin(poseAt(1, 0.2), poseAt(1.05, 0.15), {
    previousOutput: poseAt(1 - dt, 0.2 - dt),
    previousTarget: poseAt(1.05 + 2 * dt, 0.15 + 2 * dt), dt,
  });
  const start = inertial.sample(poseAt(1.05, 0.15), 0);
  near(start.localTransforms[9].translation[0], 1);
  const next = inertial.sample(poseAt(1.05 - 2 * dt, 0.15 - 2 * dt), dt);
  const velocity = (next.localTransforms[9].translation[0] - start.localTransforms[9].translation[0]) / dt;
  assert.ok(Math.abs(velocity - 1) < 0.6, `velocity jump ${velocity - 1}`);
  const yaw = (q) => 2 * Math.atan2(q[1], q[3]);
  const angularVelocity = (yaw(next.localTransforms[9].rotation) -
    yaw(start.localTransforms[9].rotation)) / dt;
  assert.ok(Math.abs(angularVelocity - 1) < 0.6, `angular velocity jump ${angularVelocity - 1}`);
  inertial.begin(next, poseAt(-1, 0.4), { previousOutput: start,
    previousTarget: poseAt(-1 - dt, 0.4 - dt), dt });
  near(inertial.sample(poseAt(-1, 0.4), 0).localTransforms[9].translation[0],
    next.localTransforms[9].translation[0]);
});

test('pose search distinguishes opposite swing velocities at the same foot position', () => {
  const candidates = buildSyntheticPoseDatabase().candidates.filter((sample) =>
    sample.clipName === 'run_forward');
  const forward = candidates.find((sample) => Math.abs(sample.timeSeconds - 0.03125) < 1e-6);
  const backward = candidates.find((sample) => Math.abs(sample.timeSeconds - 0.21875) < 1e-6);
  assert.ok(forward.features.leftFootVelocity[2] * backward.features.leftFootVelocity[2] < 0);
  assert.ok(Math.abs(forward.features.leftFootPosition[2] - backward.features.leftFootPosition[2]) < 0.01);
  const search = new PoseSearch(new PoseDatabase([forward, backward]));
  const result = search.search(forward.features);
  assert.equal(result.results[0].candidate.id, forward.id);
  assert.ok(result.results[1].velocityCost > result.results[0].velocityCost + 0.1);
});

test('rotation of query and complete database preserves search ranking and costs', () => {
  const candidateNames = ['run_forward', 'run_right'];
  let baseline = null;
  for (const degrees of [0, 45, 90, 180]) {
    const radians = degrees * Math.PI / 180;
    const rotate = ([x, y, z]) => [x * Math.cos(radians) + z * Math.sin(radians), y,
      -x * Math.sin(radians) + z * Math.cos(radians)];
    const candidates = candidateNames.map((name) => {
      const pose = sampleAnimationClip(syntheticClips[name], syntheticRig, 0.06);
      const local = [...pose.localTransforms];
      local[0] = createTransform([0, 0, 0], [0, Math.sin(radians / 2), 0,
        Math.cos(radians / 2)]);
      const turned = createPose(syntheticRig, local);
      const velocity = rotate(name.endsWith('right') ? [4.5, 0, 0] : [0, 0, 4.5]);
      const trajectory = [0.2].map((time) => ({ position: velocity.map((v) => v * time),
        velocity, facing: degrees + (name.endsWith('right') ? 90 : 0) }));
      const features = extractPoseFeatures({ pose: turned, rootVelocity: velocity, trajectory });
      return { id: name, clipName: name, timeSeconds: 0.06, pose: turned, features };
    });
    const result = new PoseSearch(new PoseDatabase(candidates)).search(candidates[0].features);
    const current = result.results.map(({ candidate, cost }) => [candidate.id, cost]);
    if (!baseline) baseline = current;
    else current.forEach(([name, cost], index) => {
      assert.equal(name, baseline[index][0]); near(cost, baseline[index][1]);
    });
  }
});

test('run/shoot/hit compose and life override owns the final pose; impact phase chooses landing clip', () => {
  const action = (name, state, sequence = 1) => ({
    [name]: { active: true, state, sequence, start_tick: 0 },
  });
  const withChannels = (channels, phase = 'run') => frame(0,
    { locomotion_phase: phase, action_channels: channels });
  const base = new CharacterAnimationRuntime().update(frame(0)).pose;
  const hit = new CharacterAnimationRuntime().update(withChannels(action('additive_reaction', 'hit')));
  const shootHit = new CharacterAnimationRuntime().update(withChannels({
    ...action('upper_body_action', 'shoot'), ...action('additive_reaction', 'hit'),
  }));
  assert.notDeepEqual(hit.pose.localTransforms[9].rotation, base.localTransforms[9].rotation);
  assert.notDeepEqual(shootHit.pose.localTransforms[9].rotation, hit.pose.localTransforms[9].rotation);
  const death = new CharacterAnimationRuntime().update(withChannels(action('life_override', 'death')));
  const shootHitDeath = new CharacterAnimationRuntime().update(withChannels({
    ...action('upper_body_action', 'shoot'), ...action('additive_reaction', 'hit'),
    ...action('life_override', 'death'),
  }));
  assert.deepEqual(shootHitDeath.pose.localTransforms[9].rotation,
    death.pose.localTransforms[9].rotation);
  const runtime = new CharacterAnimationRuntime();
  const falling = runtime.update(frame(0, { grounded: false, locomotion_phase: 'falling' }));
  const landing = runtime.update(frame(1, { locomotion_phase: 'heavy_land' }));
  assert.equal(falling.selectedClip, 'falling');
  assert.equal(landing.selectedClip, 'heavy_land');
});

test('two-bone IK clamps unreachable goals and maintains knee orientation under small target noise', () => {
  const pose = sampleAnimationClip(syntheticClips.idle, syntheticRig, 0);
  const far = solveTwoBoneLeg(pose, 'left', [-0.2, -20, 0]);
  assert.equal(far.reachable, false);
  assert.ok(Number.isFinite(far.error) && far.error > 1);
  const knees = [[-0.2, -0.77, 0.14], [-0.2, -0.77, 0.141],
    [-0.2, -0.77, 0.139]].map((target) => {
    const solved = solveTwoBoneLeg(pose, 'left', target);
    return worldTransforms(solved.pose)[syntheticRig.indexByName.get('left_calf')].translation;
  });
  for (let i = 1; i < knees.length; i++) {
    assert.ok(Math.hypot(...knees[i].map((v, axis) => v - knees[i - 1][axis])) < 0.01);
  }
});

test('initial remote shoot pose uses timeline progress instead of restarting at zero', () => {
  const snapshot = (progress) => frame(0, { action_channels: {
    upper_body_action: { active: true, state: 'shoot', sequence: 1,
      start_tick: 0, end_tick: 9, normalized_progress: progress },
  } });
  const before = new CharacterAnimationRuntime().update(snapshot(0));
  const continuing = new CharacterAnimationRuntime().update(snapshot(1 / 3));
  const baseline = new CharacterAnimationRuntime().update(frame(0));
  assert.deepEqual(before.pose.localTransforms[9].rotation,
    baseline.pose.localTransforms[9].rotation);
  assert.notDeepEqual(continuing.pose.localTransforms[9].rotation,
    baseline.pose.localTransforms[9].rotation);
});
