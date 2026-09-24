from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, field_validator

from aster_game.game.movement.state import RequestedGait, RotationMode


class WireModel(BaseModel):
    model_config = ConfigDict(extra="forbid", populate_by_name=True)


class HelloMessage(WireModel):
    type: Literal["hello"]
    protocol_version: int = Field(ge=1)


class JoinGameMessage(WireModel):
    type: Literal["join_game"]
    room_id: str | None = Field(default=None, min_length=1, max_length=64)
    player_name: str = Field(default="Player", min_length=1, max_length=24)

    @field_validator("player_name")
    @classmethod
    def normalize_player_name(cls, value: str) -> str:
        normalized = value.strip()
        if not normalized:
            raise ValueError("player_name must not be blank")
        return normalized


class InputMessage(WireModel):
    type: Literal["input"]
    sequence: int = Field(ge=0, le=9_007_199_254_740_991)
    client_tick: int = Field(ge=0, le=9_007_199_254_740_991)
    move_x: float = Field(default=0.0, ge=-1.0, le=1.0, allow_inf_nan=False)
    move_z: float = Field(default=0.0, ge=-1.0, le=1.0, allow_inf_nan=False)
    jump: bool = False
    requested_gait: RequestedGait = RequestedGait.RUN
    view_yaw: float = Field(default=0.0, ge=-180.0, le=180.0, allow_inf_nan=False)
    view_pitch: float = Field(default=0.0, ge=-89.0, le=89.0, allow_inf_nan=False)
    rotation_mode: RotationMode = RotationMode.ORIENT_TO_MOVEMENT


class AttackMessage(WireModel):
    type: Literal["attack"]


class RespawnMessage(WireModel):
    type: Literal["respawn"]


class PingMessage(WireModel):
    type: Literal["ping"]
    nonce: int | str | None = None


ClientMessage: TypeAlias = Annotated[
    HelloMessage | JoinGameMessage | InputMessage | AttackMessage | RespawnMessage | PingMessage,
    Field(discriminator="type"),
]
CLIENT_MESSAGE_ADAPTER = TypeAdapter(ClientMessage)


class MovementTuning(WireModel):
    walk_speed: float = Field(gt=0.0)
    run_speed: float = Field(gt=0.0)
    sprint_speed: float = Field(gt=0.0)
    air_control: float = Field(ge=0.0, le=1.0)
    ground_acceleration: float = Field(gt=0.0)
    braking_deceleration: float = Field(ge=0.0)
    ground_friction: float = Field(ge=0.0)
    ground_directional_friction: float = Field(ge=0.0)
    turning_deceleration: float = Field(ge=0.0)
    pivot_braking_multiplier: float = Field(ge=1.0)
    air_acceleration: float = Field(ge=0.0)
    air_max_speed: float = Field(gt=0.0)
    max_rotation_speed: float = Field(gt=0.0)
    rotation_acceleration: float = Field(gt=0.0)
    rotation_deceleration: float = Field(gt=0.0)
    walk_acceleration_curve: tuple[tuple[float, float], ...]
    run_acceleration_curve: tuple[tuple[float, float], ...]
    sprint_acceleration_curve: tuple[tuple[float, float], ...]
    braking_curve: tuple[tuple[float, float], ...]
    turn_speed_curve: tuple[tuple[float, float], ...]
    turn_in_place_threshold: float = Field(gt=0.0, le=180.0)
    pivot_angle_threshold: float = Field(ge=90.0, le=180.0)
    jump_speed: float = Field(gt=0.0)
    gravity: float = Field(gt=0.0)
    max_fall_speed: float = Field(gt=0.0)
    apex_velocity_threshold: float = Field(ge=0.0)
    jump_cooldown_seconds: float = Field(ge=0.0)
    landing_soft_velocity: float = Field(ge=0.0)
    landing_heavy_velocity: float = Field(gt=0.0)
    landing_recovery_seconds: float = Field(ge=0.0)
    max_walkable_slope: float = Field(gt=0.0, le=89.0)
    ground_probe_radius: float = Field(gt=0.0)
    ground_probe_depth: float = Field(gt=0.0)
    ground_probe_start_offset: float = Field(ge=0.0)
    ground_snap_distance: float = Field(ge=0.0)
    ground_grace_distance: float = Field(ge=0.0)
    ground_grace_ticks: int = Field(ge=0)
    character_step_height: float = Field(ge=0.0)
    character_radius: float = Field(gt=0.0)
    character_cylinder_height: float = Field(gt=0.0)


class CollisionPlane(WireModel):
    name: str
    normal: tuple[float, float, float]
    constant: float


class CollisionBox(WireModel):
    name: str
    center: tuple[float, float, float]
    half_extents: tuple[float, float, float]


class CollisionRamp(WireModel):
    name: str
    center: tuple[float, float, float]
    half_extents: tuple[float, float, float]
    pitch_degrees: float


class CollisionWorldProfile(WireModel):
    version: Literal[1]
    planes: list[CollisionPlane]
    boxes: list[CollisionBox]
    ramps: list[CollisionRamp]


class WelcomeMessage(WireModel):
    type: Literal["welcome"] = "welcome"
    session_id: str
    protocol_version: int = 4
    tick_rate: int
    snapshot_interval_ticks: int
    movement_tuning: MovementTuning
    collision_world: CollisionWorldProfile


class JoinedMessage(WireModel):
    type: Literal["joined"] = "joined"
    room_id: str
    player_id: str
    entity_id: int
    tick: int


class ErrorMessage(WireModel):
    type: Literal["error"] = "error"
    code: str
    message: str


class PongMessage(WireModel):
    type: Literal["pong"] = "pong"
    nonce: int | str | None = None


class PlayerSnapshot(WireModel):
    entity_id: int
    player_id: str
    player_name: str
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    desired_velocity: tuple[float, float, float]
    desired_move_direction: tuple[float, float, float]
    current_speed: float
    horizontal_speed: float
    vertical_speed: float
    grounded: bool
    floor_normal: tuple[float, float, float]
    floor_distance: float | None
    walkable_floor: bool
    ground_contact_confirmed: bool
    ground_sample_count: int
    blocked_move_ticks: int = Field(ge=0)
    slope_angle: float
    ground_contact_point: tuple[float, float, float] | None
    ground_entity: str | None
    movement_mode: str
    actual_gait: str
    requested_gait: str
    character_yaw: float
    desired_facing_yaw: float
    angular_velocity: float
    yaw_rate: float
    view_yaw: float
    view_pitch: float
    aim_yaw: float
    aim_pitch: float
    rotation_mode: str
    locomotion_phase: str
    phase_until_tick: int
    landing_recovery_until_tick: int
    turn_angle: float
    landing_impact_velocity: float
    jump_held: bool
    action_layer: str
    life_state: str
    hit_direction: tuple[float, float, float] | None
    hit_region: str | None
    hit_strength: float
    hit_source_position: tuple[float, float, float] | None
    health: float
    max_health: float
    last_processed_input: int
    last_client_tick: int
    jump_available_tick: int


class ProjectileSnapshot(WireModel):
    projectile_id: int
    owner_entity_id: int
    position: tuple[float, float, float]


class SnapshotMessage(WireModel):
    type: Literal["snapshot"] = "snapshot"
    tick: int
    players: list[PlayerSnapshot]
    projectiles: list[ProjectileSnapshot]
