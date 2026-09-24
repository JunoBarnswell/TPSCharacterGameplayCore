from __future__ import annotations

from math import atan2, cos, degrees, hypot, radians, sin, sqrt

from aster_game.game.movement.state import Gait, LocomotionPhase, RotationMode


def normalize_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def angle_delta(target: float, current: float) -> float:
    return normalize_degrees(target - current)


def desired_motion(
    move_x: float,
    move_z: float,
    sprint: bool,
    view_yaw: float,
    walk_speed: float,
    run_speed: float,
    sprint_speed: float,
) -> tuple[tuple[float, float, float], tuple[float, float, float], Gait]:
    axis_length = hypot(move_x, move_z)
    if axis_length <= 1e-4:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), Gait.IDLE
    scale = min(1.0, axis_length) / axis_length
    local_x = move_x * scale
    local_z = move_z * scale
    if sprint:
        speed, gait = sprint_speed, Gait.SPRINT
    elif axis_length > 0.72:
        speed, gait = run_speed, Gait.RUN
    else:
        speed, gait = walk_speed, Gait.WALK
    yaw = radians(view_yaw)
    x = local_x * cos(yaw) + local_z * sin(yaw)
    z = -local_x * sin(yaw) + local_z * cos(yaw)
    velocity = (x * speed, 0.0, z * speed)
    direction = (x / hypot(x, z), 0.0, z / hypot(x, z))
    return velocity, direction, gait


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
) -> tuple[tuple[float, float], tuple[float, float]]:
    cx, cz = current
    dx, dz = desired
    if dt <= 0:
        raise ValueError("dt must be positive")
    if grounded:
        if hypot(dx, dz) <= 1e-6:
            speed = hypot(cx, cz)
            deceleration = braking_deceleration + ground_friction * speed
            amount = min(speed, deceleration * dt)
            factor = (speed - amount) / speed if speed else 0.0
            vx, vz = cx * factor, cz * factor
        else:
            change_x, change_z = dx - cx, dz - cz
            change_length = hypot(change_x, change_z)
            max_change = ground_acceleration * dt
            factor = min(1.0, max_change / change_length) if change_length else 1.0
            vx, vz = cx + change_x * factor, cz + change_z * factor
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
) -> tuple[float, float]:
    if dt <= 0:
        raise ValueError("dt must be positive")
    difference = angle_delta(desired_yaw, current_yaw)
    if abs(difference) <= 1e-5:
        target_rate = 0.0
    else:
        brake_limited_rate = sqrt(2.0 * acceleration * abs(difference))
        target_rate = (1.0 if difference > 0 else -1.0) * min(max_speed, brake_limited_rate)
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
