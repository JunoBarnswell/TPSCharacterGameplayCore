from dataclasses import dataclass, field

from panda3d.bullet import BulletCharacterControllerNode
from panda3d.core import NodePath

from aster_game.game.movement.state import (
    ActionLayer,
    CharacterMovementState,
    LifeState,
)


@dataclass(slots=True)
class TransformComponent:
    position: tuple[float, float, float]
    yaw: float = 0.0


@dataclass(slots=True)
class HealthComponent:
    max_health: float
    current_health: float
    invulnerable_until_tick: int = 0


@dataclass(slots=True)
class FallComponent:
    airborne: bool = False
    apex_reached: bool = False
    jump_started: bool = False
    start_y: float = 0.0
    start_tick: int = 0
    impact_velocity: float = 0.0
    last_vertical_velocity: float = 0.0
    landing_started_tick: int = 0


@dataclass(slots=True)
class PhysicsComponent:
    controller: BulletCharacterControllerNode
    node_path: NodePath
    attached: bool = True


@dataclass(slots=True)
class Character:
    entity_id: int
    player_id: str
    player_name: str
    transform: TransformComponent
    movement: CharacterMovementState
    health: HealthComponent
    fall: FallComponent
    physics: PhysicsComponent
    life_state: LifeState = LifeState.ALIVE
    action_layer: ActionLayer = ActionLayer.NONE
    hit_direction: tuple[float, float, float] | None = None
    hit_region: str | None = None
    hit_strength: float = 0.0
    hit_source_position: tuple[float, float, float] | None = None
    action_until_tick: int = 0
    previous_channels: tuple[str, ...] | None = None
    attack_ready_tick: int = 0


@dataclass(slots=True)
class InputCommand:
    sequence: int
    client_tick: int
    move_x: float
    move_z: float
    jump: bool
    sprint: bool
    view_yaw: float
    view_pitch: float
    rotation_mode: str


@dataclass(slots=True)
class Projectile:
    projectile_id: int
    owner_entity_id: int
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    travelled: float = 0.0


@dataclass(slots=True)
class PendingCommands:
    inputs: dict[int, InputCommand] = field(default_factory=dict)
    jump_requests: set[int] = field(default_factory=set)
    attacks: list[int] = field(default_factory=list)
    respawns: set[int] = field(default_factory=set)
