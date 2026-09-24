from __future__ import annotations

from math import atan2, cos, degrees, hypot, radians, sin, sqrt

from aster_game.game.movement.curves import ResponseCurve, evaluate_response_curve
from aster_game.game.movement.state import Gait, LocomotionPhase, RequestedGait, RotationMode

DEFAULT_ACCELERATION_CURVE: ResponseCurve = ((0.0, 1.35), (0.5, 1.0), (1.0, 0.65))
DEFAULT_BRAKING_CURVE: ResponseCurve = ((0.0, 0.6), (0.35, 1.0), (1.0, 1.35))
DEFAULT_TURN_SPEED_CURVE: ResponseCurve = ((0.0, 0.22), (0.25, 0.55), (1.0, 1.0))


def normalize_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def angle_delta(target: float, current: float) -> float:
    return normalize_degrees(target - current)


def desired_motion(
    move_x: float,
    move_z: float,
    requested_gait: RequestedGait | str,
    view_yaw: float,
    walk_speed: float,
    run_speed: float,
    sprint_speed: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    axis_length = hypot(move_x, move_z)
    if axis_length <= 1e-4:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    gait = RequestedGait(requested_gait)
    scale = min(1.0, axis_length) / axis_length
    local_x = move_x * scale
    local_z = move_z * scale
    speed = {
        RequestedGait.WALK: walk_speed,
        RequestedGait.RUN: run_speed,
        RequestedGait.SPRINT: sprint_speed,
    }[gait]
    yaw = radians(view_yaw)
    x = local_x * cos(yaw) + local_z * sin(yaw)
    z = -local_x * sin(yaw) + local_z * cos(yaw)
    velocity = (x * speed, 0.0, z * speed)
    direction_length = hypot(x, z)
    direction = (x / direction_length, 0.0, z / direction_length)
    return velocity, direction


def derive_actual_gait(
    horizontal_speed: float,
    walk_speed: float,
    run_speed: float,
    sprint_speed: float,
) -> Gait:
    if horizontal_speed <= 0.1:
        return Gait.IDLE
    if horizontal_speed < (walk_speed + run_speed) / 2.0:
        return Gait.WALK
    if horizontal_speed < (run_speed + sprint_speed) / 2.0:
        return Gait.RUN
    return Gait.SPRINT


def _move_towards(
    current: tuple[float, float], target: tuple[float, float], max_delta: float
) -> tuple[float, float]:
    dx = target[0] - current[0]
    dz = target[1] - current[1]
    distance = hypot(dx, dz)
    if distance <= max_delta or distance <= 1e-12:
        return target
    scale = max_delta / distance
    return current[0] + dx * scale, current[1] + dz * scale


def solve_horizontal_velocity(
    current: tuple[float, float],
    desired: tuple[float, float],
    dt: float,
    *,
    grounded: bool,
    ground_acceleration: float,
    braking_deceleration: float,
    ground_friction: float,
    air_acceleration: float,
    air_control: float,
    air_max_speed: float,
    ground_directional_friction: float = 9.0,
    turning_deceleration: float = 7.0,
    pivot_braking_multiplier: float = 1.75,
    pivot_angle_threshold: float = 135.0,
    acceleration_curve: ResponseCurve = DEFAULT_ACCELERATION_CURVE,
    braking_curve: ResponseCurve = DEFAULT_BRAKING_CURVE,
    reference_speed: float = 6.5,
) -> tuple[tuple[float, float], tuple[float, float]]:
    cx, cz = current
    dx, dz = desired
    if dt <= 0:
        raise ValueError("dt must be positive")

    desired_speed = hypot(dx, dz)
    if grounded:
        speed = hypot(cx, cz)
        if desired_speed <= 1e-6:
            response = evaluate_response_curve(braking_curve, speed / reference_speed)
            deceleration = braking_deceleration * response + ground_friction * speed
            amount = min(speed, deceleration * dt)
            scale = (speed - amount) / speed if speed > 0.0 else 0.0
            vx, vz = cx * scale, cz * scale
        else:
            direction_x, direction_z = dx / desired_speed, dz / desired_speed
            projection = cx * direction_x + cz * direction_z
            lateral_x = cx - projection * direction_x
            lateral_z = cz - projection * direction_z
            lateral_speed = hypot(lateral_x, lateral_z)
            lateral_reduction = min(lateral_speed, ground_directional_friction * dt)
            lateral_scale = (
                (lateral_speed - lateral_reduction) / lateral_speed if lateral_speed else 0.0
            )
            vx = projection * direction_x + lateral_x * lateral_scale
            vz = projection * direction_z + lateral_z * lateral_scale

            current_speed = hypot(vx, vz)
            if current_speed > 1e-6:
                dot = max(-1.0, min(1.0, (vx * direction_x + vz * direction_z) / current_speed))
                turn_fraction = (1.0 - dot) * 0.5
                turn_decel = turning_deceleration * turn_fraction
                if dot <= cos(radians(pivot_angle_threshold)):
                    turn_decel *= pivot_braking_multiplier
                braking = min(current_speed, turn_decel * dt)
                turn_scale = (current_speed - braking) / current_speed
                vx *= turn_scale
                vz *= turn_scale

            speed_fraction = min(1.0, hypot(vx, vz) / max(reference_speed, 1e-6))
            response = evaluate_response_curve(acceleration_curve, speed_fraction)
            max_change = ground_acceleration * response * dt
            vx, vz = _move_towards((vx, vz), (dx, dz), max_change)
    else:
        change_x, change_z = dx - cx, dz - cz
        change_length = hypot(change_x, change_z)
        max_change = air_acceleration * air_control * dt
        factor = min(1.0, max_change / change_length) if change_length else 1.0
        vx, vz = cx + change_x * factor, cz + change_z * factor
        speed = hypot(vx, vz)
        if speed > air_max_speed:
            vx *= air_max_speed / speed
            vz *= air_max_speed / speed
    return (vx, vz), ((vx - cx) / dt, (vz - cz) / dt)


def solve_rotation(
    current_yaw: float,
    angular_velocity: float,
    desired_yaw: float,
    dt: float,
    *,
    max_speed: float,
    acceleration: float,
    deceleration: float,
    turn_speed_curve: ResponseCurve = DEFAULT_TURN_SPEED_CURVE,
) -> tuple[float, float]:
    if dt <= 0:
        raise ValueError("dt must be positive")
    difference = angle_delta(desired_yaw, current_yaw)
    turn_fraction = min(1.0, abs(difference) / 180.0)
    effective_max_speed = max_speed * evaluate_response_curve(turn_speed_curve, turn_fraction)
    if abs(difference) <= 1e-5:
        target_rate = 0.0
    else:
        brake_limited_rate = sqrt(2.0 * acceleration * abs(difference))
        target_rate = (1.0 if difference > 0 else -1.0) * min(
            effective_max_speed, brake_limited_rate
        )
    limit = acceleration if angular_velocity * target_rate >= 0 else deceleration
    delta_rate = max(-limit * dt, min(limit * dt, target_rate - angular_velocity))
    next_rate = angular_velocity + delta_rate
    applied_delta = next_rate * dt
    if abs(applied_delta) > abs(difference) and next_rate * difference > 0:
        return normalize_degrees(desired_yaw), 0.0
    return normalize_degrees(current_yaw + applied_delta), next_rate


def desired_facing_yaw(
    rotation_mode: RotationMode,
    view_yaw: float,
    move_x: float,
    move_z: float,
) -> float | None:
    if rotation_mode in (RotationMode.AIM, RotationMode.STRAFE):
        return normalize_degrees(view_yaw)
    if hypot(move_x, move_z) <= 1e-4:
        return None
    yaw = radians(view_yaw)
    x = move_x * cos(yaw) + move_z * sin(yaw)
    z = -move_x * sin(yaw) + move_z * cos(yaw)
    return normalize_degrees(degrees(atan2(x, z)))


def landing_classification(
    impact_velocity: float,
    soft_threshold: float,
    heavy_threshold: float,
) -> tuple[LocomotionPhase, str]:
    if soft_threshold >= heavy_threshold:
        raise ValueError("soft landing threshold must be below heavy threshold")
    if impact_velocity < soft_threshold:
        return LocomotionPhase.SOFT_LAND, "soft"
    if impact_velocity >= heavy_threshold:
        return LocomotionPhase.HEAVY_LAND, "heavy"
    return LocomotionPhase.NORMAL_LAND, "normal"
