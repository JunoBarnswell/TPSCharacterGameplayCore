from __future__ import annotations

from math import acos, ceil, cos, degrees, hypot, radians
from typing import TYPE_CHECKING

from panda3d.core import Vec3

from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.solver import (
    angle_delta,
    derive_actual_gait,
    desired_facing_yaw,
    desired_motion,
    landing_classification,
    project_velocity_onto_ground_plane,
    solve_horizontal_velocity,
    solve_rotation,
)
from aster_game.game.movement.state import (
    CharacterMovementState,
    Gait,
    LifeState,
    LocomotionPhase,
    MovementMode,
    RequestedGait,
    RotationMode,
)

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


def _set_locomotion_phase(
    movement: CharacterMovementState,
    phase: LocomotionPhase,
    tick: int,
    duration_ticks: int = 0,
) -> None:
    if movement.locomotion_phase is not phase or (
        duration_ticks > 0 and tick >= movement.phase_until_tick
    ):
        movement.phase_start_tick = tick
        movement.phase_duration_ticks = duration_ticks
    movement.locomotion_phase = phase


class CommandSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for entity_id, command in world.pending.inputs.items():
            character = world.characters.get(entity_id)
            if character is None or character.life_state is not LifeState.ALIVE:
                continue
            movement = character.movement
            movement.move_x = command.move_x
            movement.move_z = command.move_z
            movement.requested_gait = command.requested_gait
            movement.view_yaw = command.view_yaw
            movement.view_pitch = command.view_pitch
            movement.rotation_mode = RotationMode(command.rotation_mode)
            movement.jump_requested = entity_id in world.pending.jump_requests
            movement.jump_held = command.jump
            movement.last_input_tick = world.tick_id
            movement.last_processed_input = command.sequence
            movement.last_client_tick = command.client_tick
        world.pending.inputs.clear()
        world.pending.jump_requests.clear()
        world.attack_requests.extend(world.pending.attacks)
        world.pending.attacks.clear()
        world.respawn_requests.update(world.pending.respawns)
        world.pending.respawns.clear()


class MovementSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        settings = world.settings
        input_timeout_ticks = settings.tick_rate // 2
        jump_cooldown_ticks = ceil(settings.jump_cooldown_seconds * settings.tick_rate)
        for character in world.characters.values():
            movement = character.movement
            movement.previous_position = character.transform.position
            movement.previous_horizontal_velocity = (movement.velocity[0], movement.velocity[2])
            movement.previous_desired_velocity = movement.desired_velocity
            if character.life_state is not LifeState.ALIVE:
                character.physics.controller.setLinearMovement(Vec3(0.0, 0.0, 0.0), False)
                character.physics.controller.setGravity(settings.gravity)
                movement.movement_mode = MovementMode.DISABLED
                movement.blocked_move_ticks = 0
                movement.solver_horizontal_velocity = (0.0, 0.0)
                continue
            if world.tick_id - movement.last_input_tick > input_timeout_ticks:
                movement.move_x = 0.0
                movement.move_z = 0.0
                movement.requested_gait = RequestedGait.RUN
                movement.jump_held = False

            desired, direction = desired_motion(
                movement.move_x,
                movement.move_z,
                movement.requested_gait,
                movement.view_yaw,
                settings.walk_speed,
                settings.run_speed,
                settings.sprint_speed,
            )
            if movement.grounded and movement.walkable_floor and movement.floor_normal[1] > 1e-4:
                desired = project_velocity_onto_ground_plane(desired, movement.floor_normal)
                desired_length = hypot(hypot(desired[0], desired[1]), desired[2])
                direction = (
                    tuple(component / desired_length for component in desired)
                    if desired_length > 1e-6
                    else (0.0, 0.0, 0.0)
                )
            movement.desired_velocity = desired
            movement.desired_move_direction = direction
            acceleration_curve = {
                RequestedGait.WALK: settings.walk_acceleration_curve,
                RequestedGait.RUN: settings.run_acceleration_curve,
                RequestedGait.SPRINT: settings.sprint_acceleration_curve,
            }[movement.requested_gait]
            horizontal, acceleration = solve_horizontal_velocity(
                (movement.velocity[0], movement.velocity[2]),
                (desired[0], desired[2]),
                dt,
                grounded=movement.grounded and movement.walkable_floor,
                ground_acceleration=settings.ground_acceleration,
                braking_deceleration=settings.braking_deceleration,
                ground_friction=settings.ground_friction,
                air_acceleration=settings.air_acceleration,
                air_control=settings.air_control,
                air_max_speed=settings.air_max_speed,
                ground_directional_friction=settings.ground_directional_friction,
                turning_deceleration=settings.turning_deceleration,
                pivot_braking_multiplier=settings.pivot_braking_multiplier,
                pivot_angle_threshold=settings.pivot_angle_threshold,
                acceleration_curve=acceleration_curve,
                braking_curve=settings.braking_curve,
                reference_speed=settings.sprint_speed,
            )
            movement.solver_horizontal_velocity = horizontal
            desired_horizontal_speed = hypot(desired[0], desired[2])
            if (
                movement.grounded
                and movement.ground_contact_confirmed
                and movement.walkable_floor
                and movement.ground_contact_point is not None
                and desired_horizontal_speed > 0.5
                and movement.blocked_move_ticks >= 1
            ):
                step_origin = character.transform.position
                proposed_position = (
                    step_origin[0] + horizontal[0] * dt,
                    step_origin[1],
                    step_origin[2] + horizontal[1] * dt,
                )
                step = world.physics.find_step_up_target(
                    step_origin,
                    proposed_position,
                    movement.ground_contact_point[1],
                )
                if step is not None:
                    step_position, support = step
                    character.physics.node_path.setPos(*step_position)
                    character.transform.position = step_position
                    movement.previous_position = step_position
                    movement.floor_normal = support.normal
                    movement.floor_distance = settings.ground_probe_radius
                    movement.ground_contact_point = support.position
                    movement.ground_entity = support.node_name
                    movement.slope_angle = degrees(
                        acos(max(-1.0, min(1.0, support.normal[1])))
                    )
                    movement.walkable_floor = True
                    movement.ground_sample_count = max(movement.ground_sample_count, 1)
                    movement.grounded = True
                    movement.ground_contact_confirmed = True
                    movement.movement_mode = MovementMode.GROUNDED
                    movement.blocked_move_ticks = 0
                    world.publish(
                        "step_up",
                        entity_id=character.entity_id,
                        ground_entity=support.node_name,
                        position=step_position,
                    )
            movement.acceleration = (acceleration[0], movement.acceleration[1], acceleration[1])
            character.physics.controller.setGravity(
                0.0 if movement.grounded and movement.walkable_floor else settings.gravity
            )
            character.physics.controller.setLinearMovement(
                Vec3(
                    horizontal[0],
                    -(
                        movement.floor_normal[0] * horizontal[0]
                        + movement.floor_normal[2] * horizontal[1]
                    )
                    / movement.floor_normal[1]
                    if movement.grounded
                    and movement.walkable_floor
                    and movement.floor_normal[1] > 1e-4
                    else 0.0,
                    horizontal[1],
                ),
                False,
            )

            facing = desired_facing_yaw(
                movement.rotation_mode,
                movement.view_yaw,
                movement.move_x,
                movement.move_z,
            )
            view_delta = angle_delta(movement.view_yaw, movement.character_yaw)
            if facing is None and abs(view_delta) >= settings.turn_in_place_threshold:
                facing = movement.view_yaw
                angle = abs(view_delta)
                if movement.locomotion_phase is not LocomotionPhase.TURN_IN_PLACE:
                    movement.turn_angle = min(180.0, max(45.0, round(angle / 45.0) * 45.0))
                    movement.turn_direction = "right" if view_delta > 0 else "left"
                    duration = max(1, world.settings.tick_rate // 4)
                    _set_locomotion_phase(
                        movement, LocomotionPhase.TURN_IN_PLACE, world.tick_id, duration
                    )
                    movement.phase_until_tick = world.tick_id + duration
            if facing is not None:
                movement.desired_facing_yaw = facing
            yaw, angular_velocity = solve_rotation(
                movement.character_yaw,
                movement.angular_velocity,
                movement.desired_facing_yaw,
                dt,
                max_speed=settings.max_rotation_speed,
                acceleration=settings.rotation_acceleration,
                deceleration=settings.rotation_deceleration,
                turn_speed_curve=settings.turn_speed_curve,
            )
            movement.character_yaw = yaw
            movement.angular_velocity = angular_velocity
            movement.yaw_rate = angular_velocity
            movement.aim_yaw = angle_delta(movement.view_yaw, movement.character_yaw)
            movement.aim_pitch = movement.view_pitch
            character.transform.yaw = yaw

            if movement.jump_requested:
                can_jump = (
                    movement.grounded
                    and movement.ground_contact_confirmed
                    and movement.walkable_floor
                    and world.tick_id - movement.last_jump_tick >= jump_cooldown_ticks
                )
                if can_jump:
                    character.physics.controller.doJump()
                    character.physics.controller.setGravity(settings.gravity)
                    movement.last_jump_tick = world.tick_id
                    fall = character.fall
                    fall.airborne = True
                    fall.apex_reached = False
                    fall.jump_started = True
                    fall.start_y = character.transform.position[1]
                    fall.start_tick = world.tick_id
                    fall.last_vertical_velocity = 0.0
                    fall.impact_velocity = 0.0
                    jump_start_duration = max(1, ceil(world.settings.tick_rate * 0.2))
                    _set_locomotion_phase(
                        movement, LocomotionPhase.JUMP_START, world.tick_id, jump_start_duration
                    )
                    world.publish(
                        "jump_started",
                        entity_id=character.entity_id,
                        position=character.transform.position,
                    )
                else:
                    world.publish(
                        "command_rejected",
                        entity_id=character.entity_id,
                        reason="JUMP_INVALID",
                    )
                movement.jump_requested = False


class PhysicsStepSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        world.physics.step(dt)
        for character in world.characters.values():
            movement = character.movement
            if character.life_state is not LifeState.ALIVE:
                movement.velocity = (0.0, 0.0, 0.0)
                movement.horizontal_speed = 0.0
                movement.vertical_speed = 0.0
                movement.actual_gait = Gait.IDLE
                movement.blocked_move_ticks = 0
                continue
            position = character.physics.node_path.getPos()
            new_position = (float(position.x), float(position.y), float(position.z))
            old_position = movement.previous_position
            previous_vertical = movement.velocity[1]
            movement.velocity = (
                (new_position[0] - old_position[0]) / dt,
                (new_position[1] - old_position[1]) / dt,
                (new_position[2] - old_position[2]) / dt,
            )
            requested_x, requested_z = movement.solver_horizontal_velocity
            requested_speed = hypot(requested_x, requested_z)
            if requested_speed > 0.25:
                actual_progress = (
                    movement.velocity[0] * requested_x
                    + movement.velocity[2] * requested_z
                ) / requested_speed
                if actual_progress < requested_speed * 0.55:
                    movement.blocked_move_ticks = min(3, movement.blocked_move_ticks + 1)
                else:
                    movement.blocked_move_ticks = 0
            else:
                movement.blocked_move_ticks = 0
            movement.horizontal_speed = hypot(movement.velocity[0], movement.velocity[2])
            movement.current_speed = hypot(movement.horizontal_speed, movement.velocity[1])
            movement.vertical_speed = movement.velocity[1]
            movement.actual_gait = derive_actual_gait(
                movement.horizontal_speed,
                world.settings.walk_speed,
                world.settings.run_speed,
                world.settings.sprint_speed,
            )
            movement.acceleration = (
                movement.acceleration[0],
                (movement.velocity[1] - previous_vertical) / dt,
                movement.acceleration[2],
            )
            character.transform.position = new_position


class AirLifecycleSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for character in world.characters.values():
            if character.life_state is not LifeState.ALIVE:
                continue
            movement = character.movement
            fall = character.fall
            vertical = movement.vertical_speed
            if fall.airborne and not movement.grounded:
                if fall.jump_started and vertical > world.settings.apex_velocity_threshold:
                    if movement.locomotion_phase is LocomotionPhase.JUMP_START:
                        _set_locomotion_phase(movement, LocomotionPhase.RISING, world.tick_id)
                        world.publish("rising", entity_id=character.entity_id, tick=world.tick_id)
                elif (
                    fall.jump_started
                    and not fall.apex_reached
                    and vertical <= world.settings.apex_velocity_threshold
                ):
                    fall.apex_reached = True
                    _set_locomotion_phase(movement, LocomotionPhase.APEX, world.tick_id)
                    world.publish("apex_reached", entity_id=character.entity_id, tick=world.tick_id)
                elif (
                    fall.jump_started
                    and fall.apex_reached
                    and vertical < -world.settings.apex_velocity_threshold
                ) or (
                    not fall.jump_started
                    and vertical < -world.settings.apex_velocity_threshold
                ):
                    if movement.locomotion_phase is not LocomotionPhase.FALLING:
                        _set_locomotion_phase(movement, LocomotionPhase.FALLING, world.tick_id)
                        world.publish(
                            "fall_started", entity_id=character.entity_id, tick=world.tick_id
                        )
            elif fall.airborne and movement.grounded:
                impact_velocity = max(abs(fall.last_vertical_velocity), abs(vertical))
                fall.impact_velocity = impact_velocity
                distance = max(0.0, fall.start_y - character.transform.position[1])
                fall.airborne = False
                fall.landing_started_tick = world.tick_id
                phase, tier = landing_classification(
                    impact_velocity,
                    world.settings.landing_soft_velocity,
                    world.settings.landing_heavy_velocity,
                )
                duration = max(
                    1, ceil(world.settings.landing_recovery_seconds * world.settings.tick_rate)
                )
                _set_locomotion_phase(movement, phase, world.tick_id, duration)
                movement.phase_until_tick = world.tick_id + duration
                movement.landing_recovery_until_tick = movement.phase_until_tick
                world.publish(
                    "landing_started",
                    entity_id=character.entity_id,
                    landing_tier=tier,
                    phase=phase.value,
                    impact_velocity=impact_velocity,
                    fall_distance=distance,
                    position=character.transform.position,
                    recovery_ticks=duration,
                )
                world.publish(
                    "fall_impact",
                    entity_id=character.entity_id,
                    fall_distance=distance,
                    impact_velocity=impact_velocity,
                    impact_position=character.transform.position,
                )
                world.publish(
                    "landed",
                    entity_id=character.entity_id,
                    landing_tier=tier,
                    position=character.transform.position,
                )
                damage = max(0.0, distance - world.settings.fall_damage_start_distance)
                damage *= world.settings.fall_damage_per_meter
                if damage > 0.0:
                    world.damage_requests.append(
                        DamageRequest(
                            source_entity_id=None,
                            target_entity_id=character.entity_id,
                            damage_type=DamageType.FALL,
                            amount=damage,
                            hit_position=character.transform.position,
                            hit_direction=(0.0, 1.0, 0.0),
                        )
                    )


class LocomotionPhaseSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        phase_duration = max(1, world.settings.tick_rate // 5)
        for character in world.characters.values():
            movement = character.movement
            if character.life_state is LifeState.DEAD:
                _set_locomotion_phase(movement, LocomotionPhase.IDLE, world.tick_id)
                continue
            if movement.movement_mode is MovementMode.AIRBORNE:
                continue
            if world.tick_id < movement.landing_recovery_until_tick:
                continue
            if (
                movement.locomotion_phase
                in {
                    LocomotionPhase.SOFT_LAND,
                    LocomotionPhase.NORMAL_LAND,
                    LocomotionPhase.HEAVY_LAND,
                }
                and world.tick_id >= movement.landing_recovery_until_tick
            ):
                _set_locomotion_phase(movement, LocomotionPhase.IDLE, world.tick_id)
            if (
                movement.locomotion_phase
                in {
                    LocomotionPhase.START,
                    LocomotionPhase.STOP,
                    LocomotionPhase.PIVOT,
                    LocomotionPhase.TURN_IN_PLACE,
                }
                and world.tick_id < movement.phase_until_tick
            ):
                if movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE:
                    if abs(angle_delta(movement.desired_facing_yaw, movement.character_yaw)) < 3.0:
                        movement.phase_until_tick = world.tick_id
                    else:
                        continue
                else:
                    continue
            elif movement.locomotion_phase in {
                LocomotionPhase.START,
                LocomotionPhase.PIVOT,
                LocomotionPhase.TURN_IN_PLACE,
            }:
                _set_locomotion_phase(movement, LocomotionPhase.LOOP, world.tick_id)

            desired_speed = hypot(movement.desired_velocity[0], movement.desired_velocity[2])
            previous_speed = hypot(*movement.previous_horizontal_velocity)
            if desired_speed > 0.1:
                if previous_speed < 0.25:
                    _set_locomotion_phase(
                        movement, LocomotionPhase.START, world.tick_id, phase_duration
                    )
                    movement.phase_until_tick = world.tick_id + phase_duration
                elif previous_speed > 1.0 and movement.horizontal_speed > 1.0:
                    old_x = movement.previous_desired_velocity[0]
                    old_z = movement.previous_desired_velocity[2]
                    new_x, new_z = movement.desired_velocity[0], movement.desired_velocity[2]
                    old_desired_speed = hypot(old_x, old_z)
                    dot = (old_x * new_x + old_z * new_z) / max(
                        1e-6, old_desired_speed * desired_speed
                    )
                    pivot_dot_threshold = cos(radians(world.settings.pivot_angle_threshold))
                    if (
                        dot <= pivot_dot_threshold
                        and movement.locomotion_phase is not LocomotionPhase.PIVOT
                    ):
                        _set_locomotion_phase(
                            movement, LocomotionPhase.PIVOT, world.tick_id, phase_duration
                        )
                        movement.phase_until_tick = world.tick_id + phase_duration
                    elif movement.locomotion_phase not in {
                        LocomotionPhase.PIVOT,
                        LocomotionPhase.START,
                    }:
                        _set_locomotion_phase(movement, LocomotionPhase.LOOP, world.tick_id)
                else:
                    _set_locomotion_phase(movement, LocomotionPhase.LOOP, world.tick_id)
            elif previous_speed > 0.5:
                if movement.locomotion_phase is not LocomotionPhase.STOP:
                    _set_locomotion_phase(
                        movement, LocomotionPhase.STOP, world.tick_id, phase_duration
                    )
                    movement.phase_until_tick = world.tick_id + phase_duration
            else:
                _set_locomotion_phase(movement, LocomotionPhase.IDLE, world.tick_id)
