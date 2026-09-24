from __future__ import annotations

from math import acos, ceil, cos, degrees, hypot, radians
from typing import TYPE_CHECKING

from panda3d.core import BitMask32, Point3, Vec3

from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.solver import (
    angle_delta,
    desired_facing_yaw,
    desired_motion,
    landing_classification,
    solve_horizontal_velocity,
    solve_rotation,
)
from aster_game.game.movement.state import (
    LifeState,
    LocomotionPhase,
    MovementMode,
    RotationMode,
)

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


class CommandSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for entity_id, command in world.pending.inputs.items():
            character = world.characters.get(entity_id)
            if character is None or character.life_state is not LifeState.ALIVE:
                continue
            movement = character.movement
            movement.move_x = command.move_x
            movement.move_z = command.move_z
            movement.sprint = command.sprint
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
                movement.movement_mode = MovementMode.DISABLED
                continue
            if world.tick_id - movement.last_input_tick > input_timeout_ticks:
                movement.move_x = 0.0
                movement.move_z = 0.0
                movement.sprint = False
                movement.jump_held = False

            desired, direction, gait = desired_motion(
                movement.move_x,
                movement.move_z,
                movement.sprint,
                movement.view_yaw,
                settings.walk_speed,
                settings.run_speed,
                settings.sprint_speed,
            )
            movement.desired_velocity = desired
            movement.desired_move_direction = direction
            movement.gait = gait
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
            )
            movement.acceleration = (acceleration[0], movement.acceleration[1], acceleration[1])
            character.physics.controller.setLinearMovement(
                Vec3(horizontal[0], 0.0, horizontal[1]), False
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
                movement.turn_angle = min(180.0, max(45.0, round(angle / 45.0) * 45.0))
                if movement.locomotion_phase is not LocomotionPhase.TURN_IN_PLACE:
                    movement.locomotion_phase = LocomotionPhase.TURN_IN_PLACE
                    movement.phase_until_tick = world.tick_id + max(
                        1, world.settings.tick_rate // 4
                    )
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
                    and movement.walkable_floor
                    and world.tick_id - movement.last_jump_tick >= jump_cooldown_ticks
                )
                if can_jump:
                    character.physics.controller.doJump()
                    movement.last_jump_tick = world.tick_id
                    fall = character.fall
                    fall.airborne = True
                    fall.apex_reached = False
                    fall.jump_started = True
                    fall.start_y = character.transform.position[1]
                    fall.start_tick = world.tick_id
                    fall.last_vertical_velocity = 0.0
                    fall.impact_velocity = 0.0
                    movement.locomotion_phase = LocomotionPhase.JUMP_START
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
            movement.horizontal_speed = hypot(movement.velocity[0], movement.velocity[2])
            movement.current_speed = hypot(movement.horizontal_speed, movement.velocity[1])
            movement.vertical_speed = movement.velocity[1]
            movement.acceleration = (
                movement.acceleration[0],
                (movement.velocity[1] - previous_vertical) / dt,
                movement.acceleration[2],
            )
            character.transform.position = new_position


class GroundSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        settings = world.settings
        half_height = settings.character_radius + settings.character_cylinder_height / 2.0
        max_slope = settings.max_walkable_slope
        for character in world.characters.values():
            movement = character.movement
            if character.life_state is not LifeState.ALIVE:
                continue
            old_grounded = movement.grounded
            position = character.transform.position
            origin = Point3(position[0], position[1] + 0.2, position[2])
            end = Point3(position[0], position[1] - half_height - 0.3, position[2])
            result = world.physics.world.rayTestAll(origin, end, BitMask32.allOn())
            contacts: list[
                tuple[float, tuple[float, float, float], tuple[float, float, float], str]
            ] = []
            for hit in result.getHits():
                node = hit.getNode()
                name = node.getName()
                if name.startswith("character:"):
                    continue
                normal = hit.getHitNormal()
                contact = hit.getHitPos()
                if normal.y <= 0.0:
                    continue
                contacts.append(
                    (
                        float(hit.getHitFraction()),
                        (float(normal.x), float(normal.y), float(normal.z)),
                        (float(contact.x), float(contact.y), float(contact.z)),
                        name,
                    )
                )
            contacts.sort(key=lambda item: item[0])
            controller_grounded = character.physics.controller.isOnGround()
            if contacts:
                _, normal, contact, entity = contacts[0]
                slope = degrees(acos(max(-1.0, min(1.0, normal[1]))))
                feet_y = position[1] - half_height
                movement.floor_distance = max(0.0, feet_y - contact[1])
                movement.floor_normal = normal
                movement.ground_contact_point = contact
                movement.ground_entity = entity
                movement.slope_angle = slope
                movement.walkable_floor = slope <= max_slope
            else:
                movement.floor_distance = float("inf")
                movement.floor_normal = (0.0, 1.0, 0.0)
                movement.ground_contact_point = None
                movement.ground_entity = None
                movement.slope_angle = 0.0
                movement.walkable_floor = False
            movement.grounded = controller_grounded and movement.walkable_floor
            movement.movement_mode = (
                MovementMode.GROUNDED if movement.grounded else MovementMode.AIRBORNE
            )
            fall = character.fall
            if old_grounded and not movement.grounded and not fall.airborne:
                fall.airborne = True
                fall.apex_reached = False
                fall.jump_started = False
                fall.start_y = movement.previous_position[1]
                fall.start_tick = world.tick_id
                fall.last_vertical_velocity = movement.velocity[1]
                fall.impact_velocity = 0.0
            elif (
                not movement.grounded
                and not fall.airborne
                and movement.vertical_speed < -world.settings.apex_velocity_threshold
            ):
                fall.airborne = True
                fall.apex_reached = False
                fall.jump_started = False
                fall.start_y = movement.previous_position[1]
                fall.start_tick = world.tick_id
                fall.last_vertical_velocity = movement.vertical_speed
                fall.impact_velocity = 0.0
            elif not movement.grounded and fall.airborne:
                fall.last_vertical_velocity = movement.velocity[1]


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
                        movement.locomotion_phase = LocomotionPhase.RISING
                        world.publish("rising", entity_id=character.entity_id, tick=world.tick_id)
                elif (
                    fall.jump_started
                    and not fall.apex_reached
                    and vertical <= world.settings.apex_velocity_threshold
                ):
                    fall.apex_reached = True
                    movement.locomotion_phase = LocomotionPhase.APEX
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
                        movement.locomotion_phase = LocomotionPhase.FALLING
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
                movement.locomotion_phase = phase
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
                movement.locomotion_phase = LocomotionPhase.IDLE
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
                movement.locomotion_phase = LocomotionPhase.IDLE
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
                movement.locomotion_phase = LocomotionPhase.LOOP

            desired_speed = hypot(movement.desired_velocity[0], movement.desired_velocity[2])
            previous_speed = hypot(*movement.previous_horizontal_velocity)
            if desired_speed > 0.1:
                if previous_speed < 0.25:
                    movement.locomotion_phase = LocomotionPhase.START
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
                        movement.locomotion_phase = LocomotionPhase.PIVOT
                        movement.phase_until_tick = world.tick_id + phase_duration
                    elif movement.locomotion_phase not in {
                        LocomotionPhase.PIVOT,
                        LocomotionPhase.START,
                    }:
                        movement.locomotion_phase = LocomotionPhase.LOOP
                else:
                    movement.locomotion_phase = LocomotionPhase.LOOP
            elif previous_speed > 0.5:
                if movement.locomotion_phase is not LocomotionPhase.STOP:
                    movement.locomotion_phase = LocomotionPhase.STOP
                    movement.phase_until_tick = world.tick_id + phase_duration
            else:
                movement.locomotion_phase = LocomotionPhase.IDLE
