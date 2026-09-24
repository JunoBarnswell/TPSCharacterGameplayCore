from __future__ import annotations

from typing import TYPE_CHECKING

from aster_game.game.movement.state import (
    ActionLayer,
    Gait,
    LifeState,
    LocomotionPhase,
    MovementMode,
)

if TYPE_CHECKING:
    from aster_game.game.world import GameWorld


class RespawnSystem:
    def update(self, world: GameWorld, dt: float) -> None:
        for entity_id in sorted(world.respawn_requests):
            character = world.characters.get(entity_id)
            if character is None:
                continue
            if character.life_state is LifeState.ALIVE:
                world.publish("respawn_rejected", entity_id=entity_id, reason="ALREADY_ALIVE")
                continue
            world.respawn_character(character)
        world.respawn_requests.clear()


class StateSystem:
    """Updates orthogonal action/life channels and emits channel-change events."""

    def update(self, world: GameWorld, dt: float) -> None:
        for character in world.characters.values():
            movement = character.movement
            if character.life_state is LifeState.DEAD:
                character.action_layer = ActionLayer.DEATH
                movement.movement_mode = MovementMode.DISABLED
                movement.gait = Gait.IDLE
                movement.locomotion_phase = LocomotionPhase.IDLE
            elif world.tick_id >= character.action_until_tick and character.action_layer in {
                ActionLayer.ATTACK,
                ActionLayer.HIT_REACTION,
            }:
                character.action_layer = ActionLayer.NONE
                character.hit_direction = None
                character.hit_region = None
                character.hit_strength = 0.0
                character.hit_source_position = None

            channels = (
                movement.movement_mode.value,
                movement.gait.value,
                movement.locomotion_phase.value,
                movement.rotation_mode.value,
                character.action_layer.value,
                character.life_state.value,
            )
            if channels != character.previous_channels:
                previous = character.previous_channels
                world.publish(
                    "motion_state_changed",
                    entity_id=character.entity_id,
                    previous_channels=previous,
                    channels=channels,
                )
                character.previous_channels = channels
