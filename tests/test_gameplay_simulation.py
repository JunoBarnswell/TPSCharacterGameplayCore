from __future__ import annotations

from math import isclose

from aster_game.app.config import Settings
from aster_game.game.components import InputCommand
from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.state import LifeState, RequestedGait
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
                    sequence,
                    sequence,
                    0.0,
                    1.0,
                    False,
                    RequestedGait.SPRINT,
                    0.0,
                    0.0,
                    "orient_to_movement",
                ),
            )
            world.tick(world.settings.fixed_dt)

        assert not world.queue_input(
            character.entity_id,
            InputCommand(
                10, 10, 0.0, 1.0, False, RequestedGait.SPRINT, 0.0, 0.0, "orient_to_movement"
            ),
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
        assert character.movement.ground_contact_confirmed
        assert character.movement.ground_sample_count == 5
        assert character.movement.walkable_floor
        assert character.movement.ground_contact_point is not None
        assert character.movement.ground_entity == "arena-floor"
        assert character.movement.floor_normal[1] > 0.99

        assert world.queue_input(
            character.entity_id,
            InputCommand(
                1, 1, 0.0, 0.0, True, RequestedGait.RUN, 0.0, 0.0, "orient_to_movement"
            ),
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


def test_ground_probe_uses_configured_slope_and_five_sphere_sweeps() -> None:
    world = make_world(max_walkable_slope=37.0)
    try:
        character = world.add_player("player", "Player")
        settle(world)

        assert character.physics.controller.getMaxSlope() == 37.0
        assert character.movement.ground_contact_confirmed
        assert character.movement.grounded
        assert character.movement.ground_sample_count == 5
        assert character.movement.ground_entity == "arena-floor"
    finally:
        world.close()


def test_step_solver_moves_character_onto_walkable_step() -> None:
    world = make_world()
    try:
        character = world.add_player("player", "Player")
        start = (0.0, world.settings.spawn_height, 4.0)
        character.physics.node_path.setPos(*start)
        character.transform.position = start
        character.movement.previous_position = start
        settle(world)

        max_tick_displacement = 0.0
        for sequence in range(1, 121):
            assert world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence,
                    sequence,
                    0.0,
                    1.0,
                    False,
                    RequestedGait.RUN,
                    0.0,
                    0.0,
                    "orient_to_movement",
                ),
            )
            previous_position = character.transform.position
            world.tick(world.settings.fixed_dt)
            horizontal_displacement = sum(
                (character.transform.position[axis] - previous_position[axis]) ** 2
                for axis in (0, 2)
            ) ** 0.5
            max_tick_displacement = max(max_tick_displacement, horizontal_displacement)
            assert (
                horizontal_displacement
                <= world.settings.run_speed * world.settings.fixed_dt + 0.03
            )
            if character.movement.ground_entity == "upper-platform-step-1":
                break

        assert 4.3 < character.transform.position[2] <= 4.6
        assert character.transform.position[1] >= 1.1
        assert max_tick_displacement <= world.settings.run_speed * world.settings.fixed_dt + 0.03
        assert character.movement.grounded
        assert character.movement.ground_contact_confirmed
        assert character.movement.ground_entity == "upper-platform-step-1"
        step_y = character.transform.position[1]

        last_sequence = sequence
        for step_down_sequence in range(last_sequence + 1, last_sequence + 41):
            assert world.queue_input(
                character.entity_id,
                InputCommand(
                    step_down_sequence,
                    step_down_sequence,
                    0.0,
                    -1.0,
                    False,
                    RequestedGait.WALK,
                    0.0,
                    0.0,
                    "orient_to_movement",
                ),
            )
            world.tick(world.settings.fixed_dt)
            if character.movement.ground_entity == "arena-floor":
                break

        assert character.movement.grounded
        assert character.movement.ground_entity == "arena-floor"
        assert character.transform.position[1] < step_y - 0.2
    finally:
        world.close()


def test_walkable_slope_stays_grounded_and_moves_along_plane() -> None:
    world = make_world()
    try:
        character = world.add_player("player", "Player")
        start = (8.0, 2.0, -4.0)
        character.physics.node_path.setPos(*start)
        character.transform.position = start
        character.movement.previous_position = start
        settle(world)

        assert character.movement.grounded
        assert character.movement.ground_entity == "walkable-ramp"
        assert 19.0 <= character.movement.slope_angle <= 21.0
        start_y, start_z = character.transform.position[1], character.transform.position[2]

        for sequence in range(1, 31):
            assert world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence,
                    sequence,
                    0.0,
                    1.0,
                    False,
                    RequestedGait.WALK,
                    0.0,
                    0.0,
                    "orient_to_movement",
                ),
            )
            world.tick(world.settings.fixed_dt)

        horizontal_progress = character.transform.position[2] - start_z
        vertical_progress = character.transform.position[1] - start_y
        assert horizontal_progress > 0.45
        assert vertical_progress > 0.12
        assert 0.32 <= vertical_progress / horizontal_progress <= 0.41
        assert character.movement.grounded
        assert character.movement.ground_entity == "walkable-ramp"
    finally:
        world.close()


def test_slope_above_configured_limit_is_not_walkable() -> None:
    world = make_world(max_walkable_slope=10.0)
    try:
        character = world.add_player("player", "Player")
        start = (8.0, 2.0, -4.0)
        character.physics.node_path.setPos(*start)
        character.transform.position = start
        character.movement.previous_position = start

        world.tick(world.settings.fixed_dt)

        assert character.movement.ground_entity == "walkable-ramp"
        assert character.movement.slope_angle > world.settings.max_walkable_slope
        assert not character.movement.walkable_floor
        assert not character.movement.ground_contact_confirmed
    finally:
        world.close()


def test_ground_grace_snapshot_keeps_contact_unconfirmed_and_rejects_jump() -> None:
    world = make_world()
    try:
        character = world.add_player("grace-player", "Grace Player")
        settle(world)
        unsupported_position = (
            character.transform.position[0],
            4.0,
            character.transform.position[2],
        )
        character.physics.node_path.setPos(*unsupported_position)
        character.transform.position = unsupported_position
        character.movement.previous_position = unsupported_position
        character.movement.grounded = True
        character.movement.ground_contact_confirmed = True
        character.movement.walkable_floor = True
        character.movement.floor_distance = 0.0
        character.movement.last_grounded_tick = world.tick_id

        world.tick(world.settings.fixed_dt)
        snapshot = world.snapshot()["players"][0]
        assert snapshot["grounded"]
        assert not snapshot["ground_contact_confirmed"]
        assert snapshot["last_grounded_tick"] == world.tick_id - 1

        assert world.queue_input(
            character.entity_id,
            InputCommand(
                1,
                1,
                0.0,
                0.0,
                True,
                RequestedGait.RUN,
                0.0,
                0.0,
                "orient_to_movement",
            ),
        )
        events = world.tick(world.settings.fixed_dt)
        assert not any(event.type == "jump_started" for event in events)
        assert any(
            event.type == "command_rejected" and event.data["reason"] == "JUMP_INVALID"
            for event in events
        )
        assert character.movement.grounded
        assert not character.movement.ground_contact_confirmed
        assert character.transform.position[1] < unsupported_position[1] + 0.1
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
            InputCommand(
                1, 1, 0.0, 1.0, False, RequestedGait.SPRINT, 0.0, 0.0, "orient_to_movement"
            ),
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


def test_projectile_direction_uses_server_validated_view_ray_not_character_facing() -> None:
    world = make_world()
    try:
        attacker = world.add_player("aim-attacker", "Aim Attacker")
        target = world.add_player("aim-target", "Aim Target")
        settle(world)

        attacker_position = (-8.0, attacker.transform.position[1], -4.0)
        target_position = (-8.0, target.transform.position[1], 4.0)
        attacker.physics.node_path.setPos(*attacker_position)
        attacker.transform.position = attacker_position
        attacker.movement.previous_position = attacker_position
        attacker.movement.character_yaw = 90.0
        attacker.transform.yaw = 90.0
        target.physics.node_path.setPos(*target_position)
        target.transform.position = target_position
        target.movement.previous_position = target_position

        assert world.queue_input(
            attacker.entity_id,
            InputCommand(
                1,
                1,
                0.0,
                0.0,
                False,
                RequestedGait.RUN,
                0.0,
                0.0,
                "aim",
            ),
        )
        assert world.queue_attack(attacker.entity_id)
        events = world.tick(world.settings.fixed_dt)

        projectile = next(iter(world.projectiles.values()))
        velocity_length = sum(value * value for value in projectile.velocity) ** 0.5
        direction = tuple(value / velocity_length for value in projectile.velocity)
        assert abs(direction[0]) < 0.2
        assert direction[2] > 0.98
        fired = next(event for event in events if event.type == "attack_fired")
        assert all(
            isclose(actual, expected, abs_tol=1e-9)
            for actual, expected in zip(direction, fired.data["aim_direction"], strict=True)
        )
        assert not isclose(direction[0], 1.0, abs_tol=0.1)
    finally:
        world.close()


def test_projectile_180_degree_turn_uses_view_yaw_and_ignores_its_owner() -> None:
    world = make_world(projectile_damage=100.0)
    try:
        attacker = world.add_player("turn-attacker", "Turn Attacker")
        target = world.add_player("turn-target", "Turn Target")
        settle(world)

        attacker_position = (-8.0, attacker.transform.position[1], -4.0)
        target_position = (-8.0, target.transform.position[1], -10.0)
        attacker.physics.node_path.setPos(*attacker_position)
        attacker.transform.position = attacker_position
        attacker.movement.previous_position = attacker_position
        attacker.movement.character_yaw = 0.0
        attacker.transform.yaw = 0.0
        target.physics.node_path.setPos(*target_position)
        target.transform.position = target_position
        target.movement.previous_position = target_position

        assert world.queue_input(
            attacker.entity_id,
            InputCommand(
                1,
                1,
                0.0,
                0.0,
                False,
                RequestedGait.RUN,
                180.0,
                0.0,
                "aim",
            ),
        )
        assert world.queue_attack(attacker.entity_id)
        first_tick_events = world.tick(world.settings.fixed_dt)

        projectile = next(iter(world.projectiles.values()))
        assert projectile.position[2] < attacker_position[2]
        assert not any(event.type == "projectile_impact" for event in first_tick_events)

        events = list(first_tick_events)
        for _ in range(30):
            events.extend(world.tick(world.settings.fixed_dt))
            if target.health.current_health < target.health.max_health:
                break

        assert target.health.current_health < target.health.max_health
        assert any(event.type == "hit" for event in events)
        assert any(event.type == "damage" for event in events)
        assert not any(
            event.type == "hit" and event.data["target_entity_id"] == attacker.entity_id
            for event in events
        )
    finally:
        world.close()
