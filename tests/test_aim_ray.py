import math

import pytest

from aster_game.game.aim_ray import AimRay


def test_view_yaw_and_pitch_build_a_unit_server_aim_ray() -> None:
    ray = AimRay.from_view((1.0, 2.0, 3.0), 90.0, 30.0, 10.0, 0.5)

    assert ray.origin == pytest.approx((1.0, 2.5, 3.0))
    assert ray.direction == pytest.approx((math.cos(math.radians(30.0)), 0.5, 0.0))
    assert ray.end == pytest.approx((1.0 + 10.0 * math.cos(math.radians(30.0)), 7.5, 3.0))
    assert math.sqrt(sum(value * value for value in ray.direction)) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("position", "yaw", "pitch", "distance", "height"),
    [
        ((0.0, 0.0, 0.0), 180.01, 0.0, 10.0, 0.5),
        ((0.0, 0.0, 0.0), 0.0, 89.01, 10.0, 0.5),
        ((0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.5),
        ((0.0, 0.0, 0.0), 0.0, 0.0, 10.0, -0.1),
    ],
)
def test_aim_ray_rejects_unvalidated_view_or_range(
    position: tuple[float, float, float],
    yaw: float,
    pitch: float,
    distance: float,
    height: float,
) -> None:
    with pytest.raises(ValueError):
        AimRay.from_view(position, yaw, pitch, distance, height)
