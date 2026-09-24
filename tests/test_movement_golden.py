from __future__ import annotations

import json
from pathlib import Path

import pytest

from aster_game.game.movement.solver import (
    derive_actual_gait,
    solve_horizontal_velocity,
    solve_rotation,
)

VECTORS = json.loads(
    (Path(__file__).parent / "fixtures" / "movement-golden-vectors.json").read_text()
)


def test_python_solver_matches_shared_movement_golden_vectors() -> None:
    tuning = VECTORS["tuning"]
    curve_for_gait = {
        "walk": tuning["walk_acceleration_curve"],
        "run": tuning["run_acceleration_curve"],
        "sprint": tuning["sprint_acceleration_curve"],
    }
    for case in VECTORS["cases"]:
        velocity, acceleration = solve_horizontal_velocity(
            tuple(case["current"]),
            tuple(case["desired"]),
            VECTORS["dt"],
            grounded=case["grounded"],
            ground_acceleration=tuning["ground_acceleration"],
            braking_deceleration=tuning["braking_deceleration"],
            ground_friction=tuning["ground_friction"],
            air_acceleration=tuning["air_acceleration"],
            air_control=tuning["air_control"],
            air_max_speed=tuning["air_max_speed"],
            ground_directional_friction=tuning["ground_directional_friction"],
            turning_deceleration=tuning["turning_deceleration"],
            pivot_braking_multiplier=tuning["pivot_braking_multiplier"],
            pivot_angle_threshold=tuning["pivot_angle_threshold"],
            acceleration_curve=curve_for_gait[case["requested_gait"]],
            braking_curve=tuning["braking_curve"],
            reference_speed=tuning["sprint_speed"],
        )
        expected = case["expected"]
        assert velocity == pytest.approx(expected["velocity"], abs=1e-10), case["name"]
        assert acceleration == pytest.approx(expected["acceleration"], abs=1e-9), case["name"]
        assert derive_actual_gait(
            (velocity[0] ** 2 + velocity[1] ** 2) ** 0.5,
            tuning["walk_speed"],
            tuning["run_speed"],
            tuning["sprint_speed"],
        ).value == expected["actual_gait"]
        movement_mode = "grounded" if case["grounded"] else "airborne"
        assert movement_mode == expected["movement_mode"]

    rotation = VECTORS["rotation"]
    yaw, angular_velocity = solve_rotation(
        rotation["current_yaw"],
        rotation["angular_velocity"],
        rotation["desired_yaw"],
        VECTORS["dt"],
        max_speed=tuning["max_rotation_speed"],
        acceleration=tuning["rotation_acceleration"],
        deceleration=tuning["rotation_deceleration"],
        turn_speed_curve=tuning["turn_speed_curve"],
    )
    assert yaw == pytest.approx(rotation["expected_yaw"], abs=1e-10)
    assert angular_velocity == pytest.approx(rotation["expected_angular_velocity"], abs=1e-10)
