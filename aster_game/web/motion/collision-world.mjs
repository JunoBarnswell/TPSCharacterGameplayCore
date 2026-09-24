const CONTACT_EPSILON = 1e-5;

function normalForRamp(pitchDegrees) {
  const pitch = pitchDegrees * Math.PI / 180;
  return [0, Math.cos(pitch), Math.sin(pitch)];
}

function rampHeightAt(ramp, x, z) {
  const normal = normalForRamp(ramp.pitch_degrees);
  const [cx, cy, cz] = ramp.center;
  const [, halfY, halfZ] = ramp.half_extents;
  const pitch = ramp.pitch_degrees * Math.PI / 180;
  const topY = cy + Math.cos(pitch) * halfY;
  const topZ = cz + Math.sin(pitch) * halfY;
  const y = topY - normal[2] / normal[1] * (z - topZ);
  const dy = y - cy;
  const dz = z - cz;
  const localZ = -Math.sin(pitch) * dy + Math.cos(pitch) * dz;
  if (Math.abs(x - cx) > ramp.half_extents[0] || Math.abs(localZ) > halfZ) return null;
  return { y, normal, name: ramp.name };
}

function sweptPointAabb(startX, startZ, deltaX, deltaZ, minX, maxX, minZ, maxZ) {
  let entry = 0;
  let exit = 1;
  let normal = [0, 0];
  for (const [start, delta, minimum, maximum, axis] of [
    [startX, deltaX, minX, maxX, 0],
    [startZ, deltaZ, minZ, maxZ, 1],
  ]) {
    if (Math.abs(delta) <= CONTACT_EPSILON) {
      if (start < minimum || start > maximum) return null;
      continue;
    }
    const first = (minimum - start) / delta;
    const second = (maximum - start) / delta;
    const near = Math.min(first, second);
    const far = Math.max(first, second);
    if (near > entry) {
      entry = near;
      normal = axis === 0
        ? [delta > 0 ? -1 : 1, 0]
        : [0, delta > 0 ? -1 : 1];
    }
    exit = Math.min(exit, far);
    if (entry > exit) return null;
  }
  if (exit < 0 || entry > 1) return null;
  return { fraction: Math.max(0, entry), normal };
}

function isWalkable(normal, maxSlopeDegrees) {
  return normal[1] > 0 &&
    Math.acos(Math.max(-1, Math.min(1, normal[1]))) * 180 / Math.PI <= maxSlopeDegrees;
}

export class CollisionWorld {
  constructor(profile) {
    if (profile?.version !== 1 || !Array.isArray(profile.planes) ||
        !Array.isArray(profile.boxes) || !Array.isArray(profile.ramps)) {
      throw new TypeError("collision world profile must be version 1 with planes, boxes and ramps");
    }
    this.planes = profile.planes.map((plane) => ({ ...plane }));
    this.boxes = profile.boxes.map((box) => ({ ...box }));
    this.ramps = profile.ramps.map((ramp) => ({ ...ramp }));
  }

  #groundSamples(position, tuning) {
    const radius = tuning.character_radius;
    const halfHeight = radius + tuning.character_cylinder_height / 2;
    const feetY = position[1] - halfHeight;
    const probeRadius = tuning.ground_probe_radius;
    const probeOffset = radius * 0.9;
    const probes = [
      ["center", 0, 0],
      ["front", 0, probeOffset],
      ["back", 0, -probeOffset],
      ["right", probeOffset, 0],
      ["left", -probeOffset, 0],
    ];
    const samples = [];
    for (const [probe, offsetX, offsetZ] of probes) {
      const x = position[0] + offsetX;
      const z = position[2] + offsetZ;
      const surfaces = [];
      for (const plane of this.planes) {
        if (plane.normal[1] <= 1e-8) continue;
        const y = (plane.constant - plane.normal[0] * x - plane.normal[2] * z) /
          plane.normal[1];
        surfaces.push({ y, normal: plane.normal, name: plane.name });
      }
      for (const box of this.boxes) {
        if (Math.abs(x - box.center[0]) > box.half_extents[0] ||
            Math.abs(z - box.center[2]) > box.half_extents[2]) continue;
        surfaces.push({
          y: box.center[1] + box.half_extents[1],
          normal: [0, 1, 0],
          name: box.name,
        });
      }
      for (const ramp of this.ramps) {
        const surface = rampHeightAt(ramp, x, z);
        if (surface) surfaces.push(surface);
      }
      const highest = surfaces
        .filter(({ y }) => y <= feetY + tuning.ground_probe_start_offset + probeRadius &&
          y >= feetY - tuning.ground_probe_depth - probeRadius)
        .sort((a, b) => b.y - a.y)[0];
      if (highest) {
        samples.push({
          probe,
          node_name: highest.name,
          distance: feetY - highest.y,
          normal: highest.normal,
          position: [x, highest.y, z],
        });
      }
    }
    return samples;
  }

  #firstHorizontalHit(position, delta, halfHeight, radius) {
    let closest = null;
    for (const box of this.boxes) {
      const minimumY = box.center[1] - box.half_extents[1];
      const maximumY = box.center[1] + box.half_extents[1];
      if (maximumY <= position[1] - halfHeight + CONTACT_EPSILON ||
          minimumY >= position[1] + halfHeight - CONTACT_EPSILON) continue;
      const hit = sweptPointAabb(
        position[0], position[2], delta[0], delta[1],
        box.center[0] - box.half_extents[0] - radius,
        box.center[0] + box.half_extents[0] + radius,
        box.center[2] - box.half_extents[2] - radius,
        box.center[2] + box.half_extents[2] + radius,
      );
      if (hit && (!closest || hit.fraction < closest.fraction)) {
        closest = { ...hit, box };
      }
    }
    return closest;
  }

  #overlapsBox(position, halfHeight, radius) {
    return this.boxes.some((box) => {
      const bottom = position[1] - halfHeight;
      const top = position[1] + halfHeight;
      const overlapsY = top > box.center[1] - box.half_extents[1] + CONTACT_EPSILON &&
        bottom < box.center[1] + box.half_extents[1] - CONTACT_EPSILON;
      return overlapsY &&
        Math.abs(position[0] - box.center[0]) < box.half_extents[0] + radius - CONTACT_EPSILON &&
        Math.abs(position[2] - box.center[2]) < box.half_extents[2] + radius - CONTACT_EPSILON;
    });
  }

  #tryStep(state, position, delta, tuning) {
    if (!state.grounded || Number(state.blocked_move_ticks ?? 0) < 1 ||
        tuning.character_step_height <= 0) return null;
    const distance = Math.hypot(delta[0], delta[1]);
    if (distance <= CONTACT_EPSILON) return null;
    const directionX = delta[0] / distance;
    const directionZ = delta[1] / distance;
    const stepForward = tuning.character_radius + tuning.ground_probe_radius;
    const candidate = [
      position[0] + delta[0] + directionX * stepForward,
      position[1] + tuning.character_step_height,
      position[2] + delta[1] + directionZ * stepForward,
    ];
    const halfHeight = tuning.character_radius + tuning.character_cylinder_height / 2;
    const currentFloorY = state.ground_contact_point?.[1] ?? position[1] - halfHeight;
    const minimumNormalY = Math.cos(tuning.max_walkable_slope * Math.PI / 180);
    const candidateSamples = this.#groundSamples(candidate, tuning);
    const center = candidateSamples.find((sample) => sample.probe === "center");
    const supportCandidates = candidateSamples.filter((sample) => {
      const rise = sample.position[1] - currentFloorY;
      return sample.distance >= -tuning.ground_probe_radius &&
        sample.distance <= tuning.ground_snap_distance &&
        sample.normal[1] >= minimumNormalY && rise > 0.02 &&
        rise <= tuning.character_step_height + 0.02;
    });
    const support = supportCandidates.sort((a, b) => b.position[1] - a.position[1])[0];
    if (!support) return null;
    const target = [candidate[0], support.position[1] + halfHeight, candidate[2]];
    if (this.#overlapsBox(target, halfHeight, tuning.character_radius)) return null;
    if (center && center.position[1] > support.position[1]) return null;
    return { position: target, support };
  }

  moveCharacter(state, targetPosition, targetVelocity, tuning, dt) {
    if (!(dt > 0)) throw new RangeError("dt must be positive");
    const start = [...state.position];
    const desiredDelta = [targetPosition[0] - start[0], targetPosition[2] - start[2]];
    const halfHeight = tuning.character_radius + tuning.character_cylinder_height / 2;
    const step = this.#tryStep(state, start, desiredDelta, tuning);
    let position;
    let resolvedDelta;
    let blocked = false;
    if (step) {
      position = step.position;
      resolvedDelta = [position[0] - start[0], position[2] - start[2]];
    } else {
      position = [start[0], targetPosition[1], start[2]];
      let remaining = desiredDelta;
      for (let iteration = 0; iteration < 3; iteration++) {
        const hit = this.#firstHorizontalHit(position, remaining, halfHeight, tuning.character_radius);
        if (!hit) {
          position[0] += remaining[0];
          position[2] += remaining[1];
          break;
        }
        blocked = true;
        const distance = Math.hypot(...remaining);
        const safeFraction = Math.max(0, hit.fraction - (distance > 0 ? 0.002 / distance : 0));
        position[0] += remaining[0] * safeFraction;
        position[2] += remaining[1] * safeFraction;
        const leftover = [remaining[0] * (1 - safeFraction), remaining[1] * (1 - safeFraction)];
        const intoWall = leftover[0] * hit.normal[0] + leftover[1] * hit.normal[1];
        remaining = intoWall < 0
          ? [leftover[0] - intoWall * hit.normal[0], leftover[1] - intoWall * hit.normal[1]]
          : leftover;
        if (Math.hypot(...remaining) <= CONTACT_EPSILON) break;
      }
      position[1] = targetPosition[1];
      resolvedDelta = [position[0] - start[0], position[2] - start[2]];
    }

    const intendedSpeed = Math.hypot(targetVelocity[0], targetVelocity[2]);
    const actualVelocity = [resolvedDelta[0] / dt, resolvedDelta[1] / dt];
    const actualProgress = intendedSpeed > CONTACT_EPSILON
      ? (actualVelocity[0] * targetVelocity[0] + actualVelocity[1] * targetVelocity[2]) /
        intendedSpeed
      : 0;
    const blockedMoveTicks = step ? 0 :
      blocked && intendedSpeed > 0.25 && actualProgress < intendedSpeed * 0.55
        ? Math.min(3, Number(state.blocked_move_ticks ?? 0) + 1)
        : 0;
    let velocity = [...targetVelocity];
    if (blocked && !step) {
      velocity[0] = actualVelocity[0];
      velocity[2] = actualVelocity[1];
    }

    const samples = this.#groundSamples(position, tuning);
    const center = samples.find((sample) => sample.probe === "center");
    let selected = center ?? samples.reduce(
      (best, sample) => !best || sample.distance < best.distance ? sample : best,
      null,
    );
    if (center) {
      const raised = samples.filter((sample) => sample.probe !== "center" &&
        sample.position[1] - center.position[1] > 0.02 &&
        sample.position[1] - center.position[1] <= tuning.character_step_height + 0.02 &&
        sample.distance >= -tuning.ground_probe_radius &&
        sample.distance <= tuning.ground_snap_distance);
      if (raised.length) selected = raised.sort((a, b) => b.position[1] - a.position[1])[0];
    }
    const walkable = selected ? isWalkable(selected.normal, tuning.max_walkable_slope) : false;
    const descending = velocity[1] <= 0;
    const snapped = Boolean(walkable && descending &&
      selected.distance >= -tuning.ground_probe_radius &&
      selected.distance <= tuning.ground_snap_distance);
    let grounded = snapped;
    if (snapped) {
      position[1] -= selected.distance;
      velocity[1] = 0;
    }
    const groundSampleCount = samples.filter((sample) =>
      isWalkable(sample.normal, tuning.max_walkable_slope) &&
      sample.distance >= -tuning.ground_probe_radius &&
      sample.distance <= tuning.ground_snap_distance).length;
    const landed = !state.grounded && grounded;
    const slopeAngle = selected
      ? Math.acos(Math.max(-1, Math.min(1, selected.normal[1]))) * 180 / Math.PI
      : 0;
    return {
      position,
      velocity,
      grounded,
      walkable_floor: walkable,
      ground_contact_confirmed: grounded,
      ground_contact_point: selected?.position ?? null,
      ground_entity: selected?.node_name ?? null,
      ground_sample_count: groundSampleCount,
      floor_distance: grounded ? 0 : selected?.distance ?? null,
      floor_normal: selected?.normal ?? [0, 1, 0],
      slope_angle: slopeAngle,
      movement_mode: grounded ? "grounded" : "airborne",
      blocked_move_ticks: blockedMoveTicks,
      step_up: Boolean(step),
      landed,
    };
  }
}
