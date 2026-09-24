from __future__ import annotations

from typing import TYPE_CHECKING

from aster_game.game.movement.state import (
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

    _stride_lengths = {
        Gait.WALK: 1.35,
        Gait.RUN: 2.25,
        Gait.SPRINT: 3.1,
    }

    def update(self, world: GameWorld, dt: float) -> None:
        for character in world.characters.values():
            movement = character.movement
            channels = character.action_channels
            if character.life_state is LifeState.DEAD:
                movement.movement_mode = MovementMode.DISABLED
                movement.actual_gait = Gait.IDLE
                movement.locomotion_phase = LocomotionPhase.IDLE
                movement.phase_start_tick = world.tick_id
                movement.phase_duration_ticks = 0
                movement.turn_direction = "none"
            stride_length = self._stride_lengths.get(movement.actual_gait)
            if (
                character.life_state is LifeState.ALIVE
                and movement.grounded
                and movement.ground_contact_confirmed
                and stride_length is not None
            ):
                movement.gait_phase = (
                    movement.gait_phase + movement.horizontal_speed * dt / stride_length
                ) % 1.0
            for channel in (
                channels.upper_body_action,
                channels.additive_reaction,
                channels.full_body_override,
            ):
                if (
                    channel.active
                    and channel.end_tick is not None
                    and world.tick_id >= channel.end_tick
                ):
                    channel.deactivate(world.tick_id)
            if not channels.additive_reaction.active:
                character.hit_direction = None
                character.hit_region = None
                character.hit_strength = 0.0
                character.hit_source_position = None

            channels.update_locomotion(movement, world.tick_id)
            active_channels = tuple(
                f"{channel.channel}:{channel.state}"
                for channel in channels.ordered()
                if channel.active
            )

            channels = (
                movement.movement_mode.value,
                movement.actual_gait.value,
                movement.locomotion_phase.value,
                movement.rotation_mode.value,
                *active_channels,
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
