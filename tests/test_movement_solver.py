from __future__ import annotations

from math import cos, radians, sin

import pytest

from aster_game.app.config import Settings
from aster_game.game.components import InputCommand
from aster_game.game.events import DamageRequest, DamageType
from aster_game.game.movement.solver import (
    derive_actual_gait,
    desired_motion,
    landing_classification,
    project_velocity_onto_ground_plane,
    solve_horizontal_velocity,
    solve_rotation,
)
from aster_game.game.movement.state import (
    ActionLayer,
    Gait,
    LifeState,
    LocomotionPhase,
    MovementMode,
    RequestedGait,
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
        assert 0.0 < character.movement.horizontal_speed < world.settings.sprint_speed
        assert character.movement.requested_gait is RequestedGait.SPRINT
        assert character.movement.actual_gait is not Gait.SPRINT

        previous_speed = character.movement.horizontal_speed
        world.queue_input(
            character.entity_id,
            InputCommand(
                11, 11, 0.0, 0.0, False, RequestedGait.RUN, 0.0, 0.0, "orient_to_movement"
            ),
        )
        world.tick(world.settings.fixed_dt)
        assert 0.0 < character.movement.horizontal_speed < previous_speed
        for sequence in range(12, 100):
            world.queue_input(
                character.entity_id,
                InputCommand(
                    sequence,
                    sequence,
                    0.0,
                    0.0,
                    False,
                    RequestedGait.RUN,
                    0.0,
                    0.0,
                    "orient_to_movement",
                ),
            )
            world.tick(world.settings.fixed_dt)
        assert character.movement.horizontal_speed < 0.1
    finally:
        world.close()


def test_requested_gait_is_distinct_from_speed_derived_actual_gait() -> None:
    world = make_world()
    try:
        settings = world.settings
        desired, _ = desired_motion(
            0.0,
            1.0,
            RequestedGait.SPRINT,
            0.0,
            settings.walk_speed,
            settings.run_speed,
            settings.sprint_speed,
        )
        velocity = (0.0, 0.0)
        actual_gaits = set()
        for _ in range(settings.tick_rate * 3):
            velocity, _ = solve_horizontal_velocity(
                velocity,
                (desired[0], desired[2]),
                settings.fixed_dt,
                grounded=True,
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
                acceleration_curve=settings.sprint_acceleration_curve,
                braking_curve=settings.braking_curve,
                reference_speed=settings.sprint_speed,
            )
            actual_gaits.add(
                derive_actual_gait(
                    (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5,
                    settings.walk_speed,
                    settings.run_speed,
                    settings.sprint_speed,
                )
            )

        assert actual_gaits == {Gait.WALK, Gait.RUN, Gait.SPRINT}
        assert desired[2] == settings.sprint_speed
    finally:
        world.close()


def test_directional_friction_and_pivot_braking_preserve_bounded_momentum() -> None:
    settings = Settings()
    quarter_turn, _ = solve_horizontal_velocity(
        (0.0, 6.5),
        (6.5, 0.0),
        settings.fixed_dt,
        grounded=True,
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
        acceleration_curve=settings.run_acceleration_curve,
        braking_curve=settings.braking_curve,
        reference_speed=settings.sprint_speed,
    )
    pivot, _ = solve_horizontal_velocity(
        (0.0, 6.5),
        (0.0, -6.5),
        settings.fixed_dt,
        grounded=True,
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
        acceleration_curve=settings.run_acceleration_curve,
        braking_curve=settings.braking_curve,
        reference_speed=settings.sprint_speed,
    )
    assert quarter_turn[0] > 0.0 and quarter_turn[1] > 6.0
    assert pivot[1] > 0.0
    assert (pivot[0] ** 2 + pivot[1] ** 2) ** 0.5 < (
        quarter_turn[0] ** 2 + quarter_turn[1] ** 2
    ) ** 0.5


def test_ground_plane_projection_preserves_speed_and_removes_normal_velocity() -> None:
    angle = radians(30.0)
    normal = (0.0, cos(angle), -sin(angle))
    projected = project_velocity_onto_ground_plane((0.0, 0.0, 4.0), normal)

    assert (sum(component * component for component in projected)) ** 0.5 == pytest.approx(4.0)
    normal_velocity = sum(value * axis for value, axis in zip(projected, normal, strict=True))
    assert normal_velocity == pytest.approx(0.0, abs=1e-10)
    assert projected[1] > 0.0


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
            InputCommand(
                1, 1, 0.0, 0.0, True, RequestedGait.RUN, 0.0, 0.0, "orient_to_movement"
            ),
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
            InputCommand(
                25, 25, 0.0, 1.0, False, RequestedGait.RUN, 0.0, 0.0, "orient_to_movement"
            ),
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
            world.tick(world.settings.fixed_dt)
        world.queue_input(
            character.entity_id,
            InputCommand(
                45, 45, 0.0, -1.0, False, RequestedGait.RUN, 0.0, 0.0, "orient_to_movement"
            ),
        )
        world.tick(world.settings.fixed_dt)
        assert character.movement.locomotion_phase is LocomotionPhase.PIVOT

        world.queue_input(
            character.entity_id,
            InputCommand(
                46, 46, 0.0, 0.0, False, RequestedGait.RUN, 90.0, 0.0, "orient_to_movement"
            ),
        )
        world.tick(world.settings.fixed_dt)
        assert character.movement.locomotion_phase is LocomotionPhase.TURN_IN_PLACE
        turn_snapshot = world.snapshot()["players"][0]
        assert turn_snapshot["phase_duration_ticks"] > 0
        assert 0.0 <= turn_snapshot["phase_progress"] <= 1.0
        assert turn_snapshot["turn_direction"] in {"left", "right"}
        assert 0.0 <= turn_snapshot["turn_progress"] <= 1.0
        assert abs(turn_snapshot["remaining_turn_angle"]) <= 180.0
    finally:
        world.close()
