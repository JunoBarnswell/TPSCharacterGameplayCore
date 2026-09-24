from dataclasses import dataclass, field
from enum import StrEnum

from panda3d.bullet import BulletCharacterControllerNode
from panda3d.core import NodePath


class CharacterState(StrEnum):
    IDLE = "idle"
    WALK = "walk"
    RUN = "run"
    SPRINT = "sprint"
    JUMP = "jump"
    FALL = "fall"
    HIT = "hit"
    DEAD = "dead"


@dataclass(slots=True)
class TransformComponent:
    position: tuple[float, float, float]
    yaw: float = 0.0


@dataclass(slots=True)
class MovementComponent:
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    move_x: float = 0.0
    move_z: float = 0.0
    sprint: bool = False
    jump_held: bool = False
    jump_requested: bool = False
    grounded: bool = False
    last_input_tick: int = 0
    last_processed_input: int = -1
    last_jump_tick: int = -1_000_000
    previous_position: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(slots=True)
class HealthComponent:
    max_health: float
    current_health: float
    alive: bool = True
    invulnerable_until_tick: int = 0


@dataclass(slots=True)
class FallComponent:
    is_falling: bool = False
    start_y: float = 0.0
    start_tick: int = 0
    impact_velocity: float = 0.0
    last_vertical_velocity: float = 0.0


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
    movement: MovementComponent
    health: HealthComponent
    fall: FallComponent
    physics: PhysicsComponent
    state: CharacterState = CharacterState.IDLE
    attack_ready_tick: int = 0
    hit_reaction_until_tick: int = 0


@dataclass(slots=True)
class InputCommand:
    sequence: int
    move_x: float
    move_z: float
    jump: bool
    sprint: bool
    yaw: float


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
