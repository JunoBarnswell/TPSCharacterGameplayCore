from __future__ import annotations

from math import isfinite

ResponseCurve = tuple[tuple[float, float], ...]


def evaluate_response_curve(points: ResponseCurve, value: float) -> float:
    """Evaluate a normalized, piecewise-linear response curve."""
    if len(points) < 2:
        raise ValueError("response curves require at least two points")
    x = min(1.0, max(0.0, value))
    if x <= points[0][0]:
        return points[0][1]
    for (x0, y0), (x1, y1) in zip(points, points[1:], strict=False):
        if x <= x1:
            alpha = (x - x0) / (x1 - x0)
            return y0 + (y1 - y0) * alpha
    return points[-1][1]


def validate_response_curve(name: str, points: ResponseCurve) -> None:
    if len(points) < 2 or points[0][0] != 0.0 or points[-1][0] != 1.0:
        raise ValueError(f"{name} must cover the normalized range from 0 to 1")
    previous_x = -1.0
    for x, y in points:
        if not isfinite(x) or not isfinite(y) or not 0.0 <= x <= 1.0 or y < 0.0:
            raise ValueError(f"{name} points must be finite with x in [0, 1] and y >= 0")
        if x <= previous_x:
            raise ValueError(f"{name} x coordinates must be strictly increasing")
        previous_x = x
