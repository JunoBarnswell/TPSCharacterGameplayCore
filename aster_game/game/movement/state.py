from dataclasses import dataclass
from enum import StrEnum


class MovementMode(StrEnum):
    GROUNDED = "grounded"
    AIRBORNE = "airborne"
    DISABLED = "disabled"


class Gait(StrEnum):
    IDLE = "idle"
    WALK = "walk"
    RUN = "run"
    SPRINT = "sprint"


class RequestedGait(StrEnum):
    WALK = "walk"
    RUN = "run"
    SPRINT = "sprint"


class RotationMode(StrEnum):
    ORIENT_TO_MOVEMENT = "orient_to_movement"
    STRAFE = "strafe"
    AIM = "aim"


class LocomotionPhase(StrEnum):
    IDLE = "idle"
    START = "start"
    LOOP = "loop"
    STOP = "stop"
    PIVOT = "pivot"
    TURN_IN_PLACE = "turn_in_place"
    JUMP_START = "jump_start"
    RISING = "rising"
    APEX = "apex"
    FALLING = "falling"
    SOFT_LAND = "soft_land"
    NORMAL_LAND = "normal_land"
    HEAVY_LAND = "heavy_land"


class ActionLayer(StrEnum):
    NONE = "none"
    HIT_REACTION = "hit_reaction"
    ATTACK = "attack"
    DEATH = "death"


class LifeState(StrEnum):
    ALIVE = "alive"
    DEAD = "dead"


@dataclass(slots=True)
class CharacterMovementState:
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    acceleration: tuple[float, float, float] = (0.0, 0.0, 0.0)
    desired_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    desired_move_direction: tuple[float, float, float] = (0.0, 0.0, 0.0)
    current_speed: float = 0.0
    horizontal_speed: float = 0.0
    vertical_speed: float = 0.0
    grounded: bool = False
    floor_normal: tuple[float, float, float] = (0.0, 1.0, 0.0)
    floor_distance: float = 0.0
    walkable_floor: bool = True
    ground_contact_confirmed: bool = False
    ground_sample_count: int = 0
    last_grounded_tick: int = -1
    blocked_move_ticks: int = 0
    solver_horizontal_velocity: tuple[float, float] = (0.0, 0.0)
    slope_angle: float = 0.0
    ground_contact_point: tuple[float, float, float] | None = None
    ground_entity: str | None = None
    movement_mode: MovementMode = MovementMode.AIRBORNE
    actual_gait: Gait = Gait.IDLE
    requested_gait: RequestedGait = RequestedGait.RUN
    character_yaw: float = 0.0
    desired_facing_yaw: float = 0.0
    angular_velocity: float = 0.0
    yaw_rate: float = 0.0
    view_yaw: float = 0.0
    view_pitch: float = 0.0
    aim_yaw: float = 0.0
    aim_pitch: float = 0.0
    rotation_mode: RotationMode = RotationMode.ORIENT_TO_MOVEMENT
    locomotion_phase: LocomotionPhase = LocomotionPhase.IDLE
    turn_direction: str = "none"
    move_x: float = 0.0
    move_z: float = 0.0
    jump_held: bool = False
    jump_requested: bool = False
    last_input_tick: int = 0
    last_processed_input: int = -1
    last_client_tick: int = 0
    last_jump_tick: int = -1_000_000
    previous_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    previous_horizontal_velocity: tuple[float, float] = (0.0, 0.0)
    previous_desired_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    phase_start_tick: int = 0
    phase_duration_ticks: int = 0
    phase_until_tick: int = 0
    landing_recovery_until_tick: int = 0
    turn_angle: float = 0.0
