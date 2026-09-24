from __future__ import annotations

from aster_game.app.config import Settings
from aster_game.game.components import InputCommand
from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.solver import (
    landing_classification,
    solve_horizontal_velocity,
    solve_rotation,
)
from aster_game.game.movement.state import (
    ActionLayer,
    LifeState,
    LocomotionPhase,
    MovementMode,
)
from aster_game.game.movement_system import AirLifecycleSystem
from aster_game.game.world import GameWorld
from aster_game.infrastructure.metrics import RuntimeMetrics


def make_world(**overrides: object) -> GameWorld:
    return GameWorld("motion-room", Settings(**overrides), RuntimeMetrics())


def settle(world: GameWorld, ticks: int = 30) -> None:
    for _ in range(ticks):
        world.tick(world.settings.fixed_dt)


def test_ground_acceleration_and_braking_are_rate_limited() -> None:
    world = make_world()
    try:
        character = world.add_player("p", "Pilot")
        settle(world)
        for sequence in range(1, 11):
            world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence, sequence, 0.0, 1.0, False, True, 0.0, 0.0, "orient_to_movement"
                ),
            )
            world.tick(world.settings.fixed_dt)
        assert 0.0 < character.movement.horizontal_speed < world.settings.sprint_speed

        previous_speed = character.movement.horizontal_speed
        world.queue_input(
            character.entity_id,
            InputCommand(11, 11, 0.0, 0.0, False, False, 0.0, 0.0, "orient_to_movement"),
        )
        world.tick(world.settings.fixed_dt)
        assert 0.0 < character.movement.horizontal_speed < previous_speed
        for sequence in range(12, 100):
            world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence, sequence, 0.0, 0.0, False, False, 0.0, 0.0, "orient_to_movement"
                ),
            )
            world.tick(world.settings.fixed_dt)
        assert character.movement.horizontal_speed < 0.1
    finally:
        world.close()


def test_air_control_preserves_existing_horizontal_inertia() -> None:
    velocity, acceleration = solve_horizontal_velocity(
        (4.0, 0.0),
        (0.0, 5.0),
        1.0 / 60.0,
        grounded=False,
        ground_acceleration=24.0,
        braking_deceleration=16.0,
        ground_friction=8.0,
        air_acceleration=10.0,
        air_control=0.45,
        air_max_speed=6.5,
    )
    assert velocity[0] > 3.9
    assert 0.0 < velocity[1] < 0.1
    assert acceleration[0] < 0.0


def test_rotation_solver_respects_rate_limit_and_shortest_arc() -> None:
    yaw, yaw_rate = solve_rotation(
        179.0,
        0.0,
        -179.0,
        1.0 / 60.0,
        max_speed=360.0,
        acceleration=720.0,
        deceleration=900.0,
    )
    assert 179.0 < yaw < 181.0 or -180.0 <= yaw < -179.0
    assert abs(yaw_rate) <= 360.0


def test_landing_tiers_have_distinct_motion_semantics() -> None:
    assert landing_classification(2.0, 4.0, 9.0) == (LocomotionPhase.SOFT_LAND, "soft")
    assert landing_classification(6.0, 4.0, 9.0) == (LocomotionPhase.NORMAL_LAND, "normal")
    assert landing_classification(12.0, 4.0, 9.0) == (LocomotionPhase.HEAVY_LAND, "heavy")


def test_jump_apex_fall_landing_events_are_ordered() -> None:
    world = make_world()
    try:
        character = world.add_player("p", "Pilot")
        settle(world)
        world.queue_input(
            character.entity_id,
            InputCommand(1, 1, 0.0, 0.0, True, False, 0.0, 0.0, "orient_to_movement"),
        )
        events = []
        for _ in range(100):
            events.extend(world.tick(world.settings.fixed_dt))
        types = [event.type for event in events]
        ordered = [
            types.index("jump_started"),
            types.index("rising"),
            types.index("apex_reached"),
            types.index("fall_started"),
            types.index("landing_started"),
            types.index("landed"),
        ]
        assert ordered == sorted(ordered)
        assert character.movement.locomotion_phase in {
            LocomotionPhase.IDLE,
            LocomotionPhase.START,
            LocomotionPhase.LOOP,
        }
    finally:
        world.close()


def test_walking_off_edge_starts_falling_without_fake_apex() -> None:
    world = make_world()
    try:
        character = world.add_player("p", "Pilot")
        character.movement.grounded = False
        character.movement.movement_mode = MovementMode.AIRBORNE
        character.movement.vertical_speed = -1.0
        character.fall.airborne = True

        AirLifecycleSystem().update(world, world.settings.fixed_dt)
        types = [event.type for event in world.events.drain()]
        assert "fall_started" in types
        assert "apex_reached" not in types
        assert character.movement.locomotion_phase is LocomotionPhase.FALLING
    finally:
        world.close()


def test_hit_reaction_is_an_action_layer_over_locomotion() -> None:
    world = make_world()
    try:
        character = world.add_player("p", "Pilot")
        settle(world)
        for sequence in range(1, 25):
            world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence, sequence, 0.0, 1.0, False, False, 0.0, 0.0, "orient_to_movement"
                ),
            )
            world.tick(world.settings.fixed_dt)
        assert character.movement.locomotion_phase is LocomotionPhase.LOOP
        world.damage_requests.append(
            DamageRequest(
                source_entity_id=None,
                target_entity_id=character.entity_id,
                damage_type=DamageType.ENVIRONMENT,
                amount=5.0,
                hit_direction=(0.0, 0.0, -1.0),
            )
        )
        world.queue_input(
            character.entity_id,
            InputCommand(25, 25, 0.0, 1.0, False, False, 0.0, 0.0, "orient_to_movement"),
        )
        events = world.tick(world.settings.fixed_dt)
        assert character.action_layer is ActionLayer.HIT_REACTION
        assert character.movement.locomotion_phase is LocomotionPhase.LOOP
        assert any(event.type == "hit_reaction" for event in events)
        assert character.life_state is LifeState.ALIVE
    finally:
        world.close()


def test_pivot_and_turn_in_place_are_derived_from_motion_inputs() -> None:
    world = make_world()
    try:
        character = world.add_player("p", "Pilot")
        settle(world)
        for sequence in range(1, 45):
            world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence, sequence, 0.0, 1.0, False, False, 0.0, 0.0, "orient_to_movement"
                ),
            )
            world.tick(world.settings.fixed_dt)
        world.queue_input(
            character.entity_id,
            InputCommand(45, 45, 0.0, -1.0, False, False, 0.0, 0.0, "orient_to_movement"),
        )
        world.tick(world.settings.fixed_dt)
        assert character.movement.locomotion_phase is LocomotionPhase.PIVOT

        world.queue_input(
            character.entity_id,
            InputCommand(46, 46, 0.0, 0.0, False, False, 90.0, 0.0, "orient_to_movement"),
        )
        world.tick(world.settings.fixed_dt)
        assert character.movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE
    finally:
        world.close()
