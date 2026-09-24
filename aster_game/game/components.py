from dataclasses import dataclass, field

from panda3d.bullet import BulletCharacterControllerNode
from panda3d.core import NodePath

from aster_game.game.movement.state import (
    CharacterMovementState,
    LifeState,
    RequestedGait,
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
class AnimationChannelState:
    channel: str
    state: str = "none"
    active: bool = False
    start_tick: int = 0
    end_tick: int | None = None
    sequence: int = 0
    event_id: str = ""
    blend_semantic: str = "none"

    def activate(
        self,
        state: str,
        start_tick: int,
        end_tick: int | None,
        blend_semantic: str,
    ) -> None:
        if not state or state == "none" or end_tick is not None and end_tick < start_tick:
            raise ValueError("active animation channel requires a state and valid tick window")
        self.sequence += 1
        self.state = state
        self.active = True
        self.start_tick = start_tick
        self.end_tick = end_tick
        self.event_id = f"{self.channel}:{self.sequence}"
        self.blend_semantic = blend_semantic

    def set_locomotion_state(
        self,
        state: str,
        tick: int,
        end_tick: int | None,
        blend_semantic: str,
    ) -> None:
        if (
            self.active
            and self.state == state
            and self.end_tick == end_tick
            and self.blend_semantic == blend_semantic
        ):
            return
        self.activate(state, tick, end_tick, blend_semantic)

    def deactivate(self, tick: int) -> bool:
        if not self.active:
            return False
        self.sequence += 1
        self.state = "none"
        self.active = False
        self.end_tick = tick
        self.event_id = f"{self.channel}:{self.sequence}"
        self.blend_semantic = "none"
        return True

    def snapshot(self) -> dict[str, str | int | bool | None]:
        return {
            "state": self.state,
            "active": self.active,
            "start_tick": self.start_tick,
            "end_tick": self.end_tick,
            "sequence": self.sequence,
            "event_id": self.event_id,
            "blend_semantic": self.blend_semantic,
        }


@dataclass(slots=True)
class CharacterAnimationChannels:
    locomotion: AnimationChannelState = field(
        default_factory=lambda: AnimationChannelState(
            "locomotion", "idle", True, blend_semantic="loop"
        )
    )
    upper_body_action: AnimationChannelState = field(
        default_factory=lambda: AnimationChannelState("upper_body_action")
    )
    additive_reaction: AnimationChannelState = field(
        default_factory=lambda: AnimationChannelState("additive_reaction")
    )
    full_body_override: AnimationChannelState = field(
        default_factory=lambda: AnimationChannelState("full_body_override")
    )
    life_override: AnimationChannelState = field(
        default_factory=lambda: AnimationChannelState("life_override")
    )

    def ordered(self) -> tuple[AnimationChannelState, ...]:
        return (
            self.locomotion,
            self.upper_body_action,
            self.additive_reaction,
            self.full_body_override,
            self.life_override,
        )

    def update_locomotion(self, movement: CharacterMovementState, tick: int) -> None:
        phase = movement.locomotion_phase.value
        state = f"{movement.movement_mode.value}:{movement.actual_gait.value}:{phase}"
        transition_phases = {
            "start", "stop", "pivot", "turn_in_place", "jump_start",
            "soft_land", "normal_land", "heavy_land",
        }
        blend_semantic = "transition" if phase in transition_phases else "loop"
        end_tick = (
            movement.phase_until_tick
            if movement.phase_duration_ticks > 0
            else None
        )
        self.locomotion.set_locomotion_state(state, tick, end_tick, blend_semantic)

    def clear(self, tick: int) -> None:
        for channel in self.ordered():
            channel.deactivate(tick)

    def snapshot(self) -> dict[str, dict[str, str | int | bool | None]]:
        return {channel.channel: channel.snapshot() for channel in self.ordered()}


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
    action_channels: CharacterAnimationChannels = field(default_factory=CharacterAnimationChannels)
    hit_direction: tuple[float, float, float] | None = None
    hit_region: str | None = None
    hit_strength: float = 0.0
    hit_source_position: tuple[float, float, float] | None = None
    previous_channels: tuple[str, ...] | None = None
    attack_ready_tick: int = 0


@dataclass(slots=True)
class InputCommand:
    sequence: int
    client_tick: int
    move_x: float
    move_z: float
    jump: bool
    requested_gait: RequestedGait
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
