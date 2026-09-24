from __future__ import annotations

from aster_game.app.config import Settings
from aster_game.game.components import InputCommand
from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.state import LifeState
from aster_game.game.world import GameWorld
from aster_game.infrastructure.metrics import RuntimeMetrics


def make_world(**overrides: object) -> GameWorld:
    settings = Settings(**overrides)
    return GameWorld("test-room", settings, RuntimeMetrics())


def settle(world: GameWorld, ticks: int = 30) -> None:
    for _ in range(ticks):
        world.tick(world.settings.fixed_dt)


def test_input_sequence_is_authoritative_and_movement_is_speed_limited() -> None:
    world = make_world()
    try:
        character = world.add_player("player", "Player")
        settle(world)
        start = character.transform.position

        for sequence in range(1, 11):
            assert world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence, sequence, 0.0, 1.0, False, True, 0.0, 0.0, "orient_to_movement"
                ),
            )
            world.tick(world.settings.fixed_dt)

        assert not world.queue_input(
            character.entity_id,
            InputCommand(10, 10, 0.0, 1.0, False, True, 0.0, 0.0, "orient_to_movement"),
        )
        displacement = character.transform.position[2] - start[2]
        assert displacement > 0.0
        assert displacement <= world.settings.sprint_speed * 10 * world.settings.fixed_dt + 0.15
        assert character.movement.last_processed_input == 10
    finally:
        world.close()


def test_jump_enters_air_and_returns_to_ground() -> None:
    world = make_world()
    try:
        character = world.add_player("player", "Player")
        settle(world)
        ground_y = character.transform.position[1]
        assert character.movement.grounded
        assert character.movement.walkable_floor
        assert character.movement.ground_contact_point is not None
        assert character.movement.ground_entity == "arena-floor"
        assert character.movement.floor_normal[1] > 0.99

        assert world.queue_input(
            character.entity_id,
            InputCommand(1, 1, 0.0, 0.0, True, False, 0.0, 0.0, "orient_to_movement"),
        )
        events = []
        highest_y = ground_y
        for _ in range(90):
            events.extend(world.tick(world.settings.fixed_dt))
            highest_y = max(highest_y, character.transform.position[1])

        assert highest_y > ground_y + 0.5
        assert character.movement.grounded
        assert any(event.type == "jump_started" for event in events)
        assert any(event.type == "fall_started" for event in events)
        assert any(event.type == "landing_started" for event in events)
        assert any(event.type == "landed" for event in events)
    finally:
        world.close()


def test_fall_damage_uses_damage_pipeline_and_can_kill() -> None:
    world = make_world(fall_damage_start_distance=2.0, fall_damage_per_meter=20.0)
    try:
        character = world.add_player("player", "Player")
        high_position = (character.transform.position[0], 18.0, character.transform.position[2])
        character.physics.node_path.setPos(*high_position)
        character.transform.position = high_position
        character.movement.previous_position = high_position

        events = []
        for _ in range(120):
            events.extend(world.tick(world.settings.fixed_dt))
            if character.life_state is LifeState.DEAD:
                break

        event_types = [event.type for event in events]
        assert "fall_impact" in event_types
        assert "damage" in event_types
        assert "death" in event_types
        assert character.life_state is LifeState.DEAD
        assert character.health.current_health == 0.0
    finally:
        world.close()


def test_same_tick_damage_is_capped_to_remaining_health() -> None:
    world = make_world(max_health=50.0)
    try:
        character = world.add_player("player", "Player")
        world.damage_requests.extend(
            DamageRequest(
                source_entity_id=None,
                target_entity_id=character.entity_id,
                damage_type=DamageType.ENVIRONMENT,
                amount=40.0,
            )
            for _ in range(2)
        )

        events = world.tick(world.settings.fixed_dt)
        damage_amounts = [event.data["amount"] for event in events if event.type == "damage"]

        assert damage_amounts == [40.0, 10.0]
        assert sum(damage_amounts) == 50.0
        assert character.health.current_health == 0.0
        assert character.life_state is LifeState.DEAD
        assert sum(event.type == "death" for event in events) == 1
    finally:
        world.close()


def test_projectile_hit_death_and_respawn_restore_character_state() -> None:
    world = make_world(
        max_health=40.0,
        projectile_damage=40.0,
        attack_cooldown_seconds=0.03,
    )
    try:
        attacker = world.add_player("attacker", "Attacker")
        target = world.add_player("target", "Target")
        settle(world)

        attacker_position = (-8.0, attacker.transform.position[1], -4.0)
        target_position = (-8.0, target.transform.position[1], 4.0)
        attacker.physics.node_path.setPos(*attacker_position)
        attacker.transform.position = attacker_position
        attacker.movement.previous_position = attacker_position
        target.physics.node_path.setPos(*target_position)
        target.transform.position = target_position
        target.movement.previous_position = target_position

        assert world.queue_attack(attacker.entity_id)
        events = []
        for _ in range(30):
            events.extend(world.tick(world.settings.fixed_dt))
            if target.life_state is LifeState.DEAD:
                break

        assert target.life_state is LifeState.DEAD
        assert target.health.current_health == 0.0
        assert any(event.type == "hit" for event in events)
        assert any(event.type == "damage" for event in events)
        assert any(event.type == "death" for event in events)
        assert not world.queue_input(
            target.entity_id,
            InputCommand(1, 1, 0.0, 1.0, False, True, 0.0, 0.0, "orient_to_movement"),
        )

        assert world.queue_respawn(target.entity_id)
        respawn_events = world.tick(world.settings.fixed_dt)
        assert target.life_state is LifeState.ALIVE
        assert target.health.current_health == target.health.max_health
        assert target.movement.movement_mode.value != "disabled"
        assert target.movement.last_processed_input == -1
        assert any(event.type == "respawn" for event in respawn_events)
    finally:
        world.close()
