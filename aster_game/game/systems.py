from __future__ import annotations

from math import ceil, cos, hypot, radians, sin
from typing import TYPE_CHECKING

from panda3d.core import Vec3

from aster_game.game.components import CharacterState, Projectile
from aster_game.game.events import DamageRequest, DamageType

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


class CommandSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for entity_id, command in world.pending.inputs.items():
            character = world.characters.get(entity_id)
            if character is None or not character.health.alive:
                continue
            movement = character.movement
            movement.move_x = command.move_x
            movement.move_z = command.move_z
            movement.sprint = command.sprint
            character.transform.yaw = command.yaw
            movement.jump_requested = entity_id in world.pending.jump_requests
            movement.jump_held = command.jump
            movement.last_input_tick = world.tick_id
            movement.last_processed_input = command.sequence
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
            if not character.health.alive:
                character.physics.controller.setLinearMovement(Vec3(0.0, 0.0, 0.0), False)
                continue
            if world.tick_id - movement.last_input_tick > input_timeout_ticks:
                movement.move_x = 0.0
                movement.move_z = 0.0
                movement.sprint = False
                movement.jump_held = False

            axis_length = hypot(movement.move_x, movement.move_z)
            move_x = movement.move_x
            move_z = movement.move_z
            if axis_length > 1.0:
                move_x /= axis_length
                move_z /= axis_length

            if movement.sprint:
                speed = settings.sprint_speed
            elif axis_length > 0.72:
                speed = settings.run_speed
            else:
                speed = settings.walk_speed

            yaw = radians(character.transform.yaw)
            velocity_x = speed * (move_x * cos(yaw) + move_z * sin(yaw))
            velocity_z = speed * (-move_x * sin(yaw) + move_z * cos(yaw))
            control = 1.0 if movement.grounded else settings.air_control
            character.physics.controller.setLinearMovement(
                Vec3(velocity_x * control, 0.0, velocity_z * control), False
            )

            if movement.jump_requested:
                can_jump = (
                    movement.grounded
                    and world.tick_id - movement.last_jump_tick >= jump_cooldown_ticks
                )
                if can_jump:
                    character.physics.controller.doJump()
                    movement.last_jump_tick = world.tick_id
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
            if not character.health.alive:
                character.movement.velocity = (0.0, 0.0, 0.0)
                continue
            position = character.physics.node_path.getPos()
            new_position = (float(position.x), float(position.y), float(position.z))
            old_position = character.movement.previous_position
            character.transform.position = new_position
            character.movement.velocity = (
                (new_position[0] - old_position[0]) / dt,
                (new_position[1] - old_position[1]) / dt,
                (new_position[2] - old_position[2]) / dt,
            )
            character.movement.grounded = character.physics.controller.isOnGround()
            if not character.movement.grounded and character.fall.is_falling:
                character.fall.last_vertical_velocity = character.movement.velocity[1]


class FallSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for character in world.characters.values():
            if not character.health.alive:
                continue
            fall = character.fall
            movement = character.movement
            if not movement.grounded and not fall.is_falling:
                fall.is_falling = True
                fall.start_y = movement.previous_position[1]
                fall.start_tick = world.tick_id
                fall.last_vertical_velocity = movement.velocity[1]
                world.publish(
                    "fall_started", entity_id=character.entity_id, tick=world.tick_id
                )
                continue
            if movement.grounded and fall.is_falling:
                distance = max(0.0, fall.start_y - character.transform.position[1])
                impact_velocity = max(abs(fall.last_vertical_velocity), abs(movement.velocity[1]))
                fall.is_falling = False
                world.publish(
                    "fall_impact",
                    entity_id=character.entity_id,
                    fall_distance=distance,
                    impact_velocity=impact_velocity,
                    impact_position=character.transform.position,
                )
                world.publish(
                    "landing", entity_id=character.entity_id, position=character.transform.position
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


class AttackSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        cooldown_ticks = max(
            1, ceil(world.settings.attack_cooldown_seconds * world.settings.tick_rate)
        )
        for entity_id in world.attack_requests:
            character = world.characters.get(entity_id)
            if character is None or not character.health.alive:
                world.publish("attack_rejected", entity_id=entity_id, reason="NOT_ALIVE")
                continue
            if world.tick_id < character.attack_ready_tick:
                world.publish("attack_rejected", entity_id=entity_id, reason="COOLDOWN")
                continue

            yaw = radians(character.transform.yaw)
            direction = (sin(yaw), 0.0, cos(yaw))
            muzzle_distance = (
                world.settings.character_radius + world.settings.projectile_radius + 0.18
            )
            position = character.transform.position
            origin = (
                position[0] + direction[0] * muzzle_distance,
                position[1] + 0.2,
                position[2] + direction[2] * muzzle_distance,
            )
            projectile = Projectile(
                projectile_id=world.next_projectile_id,
                owner_entity_id=entity_id,
                position=origin,
                velocity=tuple(value * world.settings.projectile_speed for value in direction),
            )
            world.next_projectile_id += 1
            world.projectiles[projectile.projectile_id] = projectile
            character.attack_ready_tick = world.tick_id + cooldown_ticks
            world.publish(
                "attack_fired",
                entity_id=entity_id,
                projectile_id=projectile.projectile_id,
                position=origin,
            )
        world.attack_requests.clear()


class ProjectileSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        expired: list[int] = []
        for projectile_id, projectile in world.projectiles.items():
            start = projectile.position
            end = tuple(start[i] + projectile.velocity[i] * dt for i in range(3))
            step_distance = hypot(
                hypot(end[0] - start[0], end[1] - start[1]), end[2] - start[2]
            )
            hit = world.physics.sweep_projectile(start, end)
            if hit is not None:
                target_id = world.entity_id_from_physics_name(hit.node_name)
                if target_id is not None and target_id != projectile.owner_entity_id:
                    target = world.characters.get(target_id)
                    if target is not None and target.health.alive:
                        world.publish(
                            "hit",
                            source_entity_id=projectile.owner_entity_id,
                            target_entity_id=target_id,
                            projectile_id=projectile_id,
                            hit_position=hit.position,
                        )
                        world.damage_requests.append(
                            DamageRequest(
                                source_entity_id=projectile.owner_entity_id,
                                target_entity_id=target_id,
                                damage_type=DamageType.PROJECTILE,
                                amount=world.settings.projectile_damage,
                                hit_position=hit.position,
                                hit_direction=tuple(
                                    value / world.settings.projectile_speed
                                    for value in projectile.velocity
                                ),
                            )
                        )
                world.publish(
                    "projectile_impact",
                    projectile_id=projectile_id,
                    position=hit.position,
                )
                expired.append(projectile_id)
                continue

            projectile.position = end
            projectile.travelled += step_distance
            if projectile.travelled >= world.settings.projectile_range:
                expired.append(projectile_id)
        for projectile_id in expired:
            world.projectiles.pop(projectile_id, None)


class DamageSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        requests, world.damage_requests = world.damage_requests, []
        for request in requests:
            target = world.characters.get(request.target_entity_id)
            if target is None or not target.health.alive:
                continue
            if world.tick_id < target.health.invulnerable_until_tick:
                continue
            previous = target.health.current_health
            applied = min(previous, max(0.0, request.amount))
            if applied <= 0.0:
                continue
            target.health.current_health = max(0.0, previous - applied)
            target.hit_reaction_until_tick = world.tick_id + max(1, world.settings.tick_rate // 5)
            world.publish(
                "damage",
                source_entity_id=request.source_entity_id,
                target_entity_id=request.target_entity_id,
                damage_type=request.damage_type.value,
                amount=applied,
                remaining_health=target.health.current_health,
                hit_position=request.hit_position,
                hit_direction=request.hit_direction,
            )
            world.publish(
                "health_changed",
                entity_id=request.target_entity_id,
                current_health=target.health.current_health,
                max_health=target.health.max_health,
            )
            if target.health.current_health <= 0.0:
                target.health.alive = False
                target.movement.velocity = (0.0, 0.0, 0.0)
                target.movement.move_x = 0.0
                target.movement.move_z = 0.0
                target.movement.jump_requested = False
                target.physics.controller.setLinearMovement(Vec3(0.0, 0.0, 0.0), False)
                world.physics.detach_character(target.physics.controller)
                target.physics.attached = False
                world.publish(
                    "death",
                    entity_id=request.target_entity_id,
                    killer_id=request.source_entity_id,
                    damage_type=request.damage_type.value,
                    tick=world.tick_id,
                )


class RespawnSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for entity_id in sorted(world.respawn_requests):
            character = world.characters.get(entity_id)
            if character is None:
                continue
            if character.health.alive:
                world.publish("respawn_rejected", entity_id=entity_id, reason="ALREADY_ALIVE")
                continue
            world.respawn_character(character)
        world.respawn_requests.clear()


class StateSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for character in world.characters.values():
            previous_state = character.state
            if not character.health.alive:
                character.state = CharacterState.DEAD
            elif world.tick_id < character.hit_reaction_until_tick:
                character.state = CharacterState.HIT
            else:
                movement = character.movement
                if not movement.grounded:
                    character.state = (
                        CharacterState.JUMP
                        if movement.velocity[1] > 0.15
                        else CharacterState.FALL
                    )
                elif hypot(movement.move_x, movement.move_z) < 0.01:
                    character.state = CharacterState.IDLE
                elif movement.sprint:
                    character.state = CharacterState.SPRINT
                elif hypot(movement.move_x, movement.move_z) > 0.72:
                    character.state = CharacterState.RUN
                else:
                    character.state = CharacterState.WALK
            if character.state != previous_state:
                world.publish(
                    "state_changed",
                    entity_id=character.entity_id,
                    previous_state=previous_state.value,
                    state=character.state.value,
                )
