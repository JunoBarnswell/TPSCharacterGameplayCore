"""System exports; implementations live in focused movement, combat, and lifecycle modules."""

from aster_game.game.combat_system import (
    AttackSystem,
    DamageSystem,
    DeathSystem,
    HealthSystem,
    ProjectileSystem,
)
from aster_game.game.lifecycle_system import RespawnSystem, StateSystem
from aster_game.game.movement.ground_system import GroundSystem
from aster_game.game.movement_system import (
    AirLifecycleSystem,
    CommandSystem,
    LocomotionPhaseSystem,
    MovementSystem,
    PhysicsStepSystem,
)

__all__ = [
    "AirLifecycleSystem",
    "AttackSystem",
    "CommandSystem",
    "DamageSystem",
    "DeathSystem",
    "GroundSystem",
    "HealthSystem",
    "LocomotionPhaseSystem",
    "MovementSystem",
    "PhysicsStepSystem",
    "ProjectileSystem",
    "RespawnSystem",
    "StateSystem",
]
