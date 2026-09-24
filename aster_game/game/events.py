from dataclasses import dataclass
from enum import StrEnum


class DamageType(StrEnum):
    PROJECTILE = "projectile"
    FALL = "fall"
    ENVIRONMENT = "environment"


@dataclass(frozen=True, slots=True)
class DamageRequest:
    source_entity_id: int | None
    target_entity_id: int
    damage_type: DamageType
    amount: float
    hit_position: tuple[float, float, float] | None = None
    hit_direction: tuple[float, float, float] | None = None


@dataclass(frozen=True, slots=True)
class GameplayEvent:
    type: str
    tick: int
    data: dict[str, object]


class EventBus:
    def __init__(self) -> None:
        self._pending: list[GameplayEvent] = []

    def publish(self, event: GameplayEvent) -> None:
        self._pending.append(event)

    def drain(self) -> list[GameplayEvent]:
        events, self._pending = self._pending, []
        return events
