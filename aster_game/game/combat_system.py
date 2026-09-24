from __future__ import annotations

from math import ceil, cos, radians, sin
from typing import TYPE_CHECKING

from panda3d.core import Vec3

from aster_game.game.components import Projectile
from aster_game.game.events import DamageRequest, DamageType, ResolvedDamage
from aster_game.game.movement.state import ActionLayer, LifeState, MovementMode

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


class AttackSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        cooldown_ticks = max(
            1, ceil(world.settings.attack_cooldown_seconds * world.settings.tick_rate)
        )
        for entity_id in world.attack_requests:
            character = world.characters.get(entity_id)
            if character is None or character.life_state is not LifeState.ALIVE:
                world.publish("attack_rejected", entity_id=entity_id, reason="NOT_ALIVE")
                continue
            if world.tick_id < character.attack_ready_tick:
                world.publish("attack_rejected", entity_id=entity_id, reason="COOLDOWN")
                continue
            yaw = radians(character.movement.character_yaw)
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
            character.action_layer = ActionLayer.ATTACK
            character.action_until_tick = world.tick_id + max(1, world.settings.tick_rate // 6)
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
            step_distance = sum((end[i] - start[i]) ** 2 for i in range(3)) ** 0.5
            hit = world.physics.sweep_projectile(start, end)
            if hit is not None:
                target_id = world.entity_id_from_physics_name(hit.node_name)
                if target_id is not None and target_id != projectile.owner_entity_id:
                    target = world.characters.get(target_id)
                    attacker = world.characters.get(projectile.owner_entity_id)
                    if target is not None and target.life_state is LifeState.ALIVE:
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
                                hit_source_position=(
                                    attacker.transform.position if attacker is not None else None
                                ),
                            )
                        )
                world.publish(
                    "projectile_impact", projectile_id=projectile_id, position=hit.position
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
    """Validates requests, computes applied amounts, and creates reaction data."""

    def update(self, world: GameWorld, dt: float) -> None:
        requests, world.damage_requests = world.damage_requests, []
        remaining_health: dict[int, float] = {}
        for request in requests:
            target = world.characters.get(request.target_entity_id)
            if target is None or target.life_state is not LifeState.ALIVE:
                continue
            if world.tick_id < target.health.invulnerable_until_tick:
                continue
            available = remaining_health.setdefault(
                target.entity_id, target.health.current_health
            )
            applied = min(available, max(0.0, request.amount))
            if applied <= 0.0:
                continue
            remaining_health[target.entity_id] = available - applied
            world.resolved_damage.append(ResolvedDamage(request, applied))
            target.hit_direction = request.hit_direction
            target.hit_region = request.hit_region or "body"
            target.hit_strength = min(1.0, applied / target.health.max_health)
            target.hit_source_position = request.hit_source_position
            target.action_layer = ActionLayer.HIT_REACTION
            target.action_until_tick = world.tick_id + max(1, world.settings.tick_rate // 5)
            side = self._hit_side(target.movement.character_yaw, request.hit_direction)
            world.publish(
                "hit_reaction",
                entity_id=target.entity_id,
                hit_side=side,
                hit_direction=request.hit_direction,
                hit_region=target.hit_region,
                hit_strength=target.hit_strength,
                source_position=request.hit_source_position,
                duration_ticks=max(1, world.settings.tick_rate // 5),
            )

    @staticmethod
    def _hit_side(yaw: float, direction: tuple[float, float, float] | None) -> str:
        if direction is None:
            return "front"
        angle = radians(yaw)
        forward_x, forward_z = sin(angle), cos(angle)
        forward_dot = -(direction[0] * forward_x + direction[2] * forward_z)
        right_dot = -(direction[0] * forward_z - direction[2] * forward_x)
        if abs(forward_dot) >= abs(right_dot):
            return "front" if forward_dot >= 0.0 else "back"
        return "right" if right_dot >= 0.0 else "left"


class HealthSystem:
    """Applies resolved damage to the health channel and queues death transitions."""

    def update(self, world: GameWorld, dt: float) -> None:
        changes, world.resolved_damage = world.resolved_damage, []
        for change in changes:
            request = change.request
            target = world.characters.get(request.target_entity_id)
            if target is None or target.life_state is not LifeState.ALIVE:
                continue
            target.health.current_health = max(
                0.0, target.health.current_health - change.applied_amount
            )
            world.publish(
                "damage",
                source_entity_id=request.source_entity_id,
                target_entity_id=request.target_entity_id,
                damage_type=request.damage_type.value,
                amount=change.applied_amount,
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
                world.death_requests.append(request)


class DeathSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        requests, world.death_requests = world.death_requests, []
        for request in requests:
            target = world.characters.get(request.target_entity_id)
            if target is None or target.life_state is LifeState.DEAD:
                continue
            target.life_state = LifeState.DEAD
            target.action_layer = ActionLayer.DEATH
            target.movement.velocity = (0.0, 0.0, 0.0)
            target.movement.acceleration = (0.0, 0.0, 0.0)
            target.movement.desired_velocity = (0.0, 0.0, 0.0)
            target.movement.move_x = 0.0
            target.movement.move_z = 0.0
            target.movement.jump_requested = False
            target.movement.movement_mode = MovementMode.DISABLED
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
