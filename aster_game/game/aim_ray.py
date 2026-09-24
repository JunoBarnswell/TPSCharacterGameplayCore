from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, radians, sin


@dataclass(frozen=True, slots=True)
class AimRay:
    origin: tuple[float, float, float]
    direction: tuple[float, float, float]
    max_distance: float

    def __post_init__(self) -> None:
        if (
            not isinstance(self.origin, tuple)
            or len(self.origin) != 3
            or not all(map(_finite, self.origin))
            or not isinstance(self.direction, tuple)
            or len(self.direction) != 3
            or not all(map(_finite, self.direction))
            or abs(sum(component * component for component in self.direction) - 1.0) > 1e-6
            or not _finite(self.max_distance)
            or self.max_distance <= 0.0
        ):
            raise ValueError("aim ray requires a finite origin, unit direction, and positive range")

    @classmethod
    def from_view(
        cls,
        position: tuple[float, float, float],
        view_yaw: float,
        view_pitch: float,
        max_distance: float,
        origin_height: float,
    ) -> AimRay:
        if (
            len(position) != 3
            or not all(map(_finite, position))
            or not _finite(view_yaw)
            or not -180.0 <= view_yaw <= 180.0
            or not _finite(view_pitch)
            or not -89.0 <= view_pitch <= 89.0
            or not _finite(origin_height)
            or origin_height < 0.0
        ):
            raise ValueError("aim view must contain validated yaw, pitch, and world position")
        yaw = radians(view_yaw)
        pitch = radians(view_pitch)
        pitch_cos = cos(pitch)
        direction = (
            sin(yaw) * pitch_cos,
            sin(pitch),
            cos(yaw) * pitch_cos,
        )
        return cls(
            origin=(position[0], position[1] + origin_height, position[2]),
            direction=direction,
            max_distance=max_distance,
        )

    @property
    def end(self) -> tuple[float, float, float]:
        return tuple(
            self.origin[axis] + self.direction[axis] * self.max_distance for axis in range(3)
        )


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and isfinite(value)
