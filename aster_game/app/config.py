from functools import lru_cache

from pydantic import Field, model_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_prefix="ASTER_GAME_", env_file=".env", extra="ignore")

    host: str = "127.0.0.1"
    port: int = Field(default=8000, ge=1, le=65535)
    tick_rate: int = Field(default=60, ge=60, le=120)
    snapshot_interval_ticks: int = Field(default=3, ge=1)
    max_rooms: int = Field(default=8, ge=1)
    max_players_per_room: int = Field(default=16, ge=2, le=16)
    heartbeat_timeout_seconds: float = Field(default=30.0, gt=1.0)
    max_outbound_messages: int = Field(default=64, ge=8)
    max_pending_attacks_per_player: int = Field(default=4, ge=1)
    arena_half_extent: float = Field(default=25.0, ge=22.0)
    walk_speed: float = Field(default=2.0, gt=0.0)
    run_speed: float = Field(default=4.5, gt=0.0)
    sprint_speed: float = Field(default=6.5, gt=0.0)
    air_control: float = Field(default=0.45, ge=0.0, le=1.0)
    ground_acceleration: float = Field(default=24.0, gt=0.0)
    braking_deceleration: float = Field(default=16.0, ge=0.0)
    ground_friction: float = Field(default=8.0, ge=0.0)
    air_acceleration: float = Field(default=10.0, ge=0.0)
    air_max_speed: float = Field(default=6.5, gt=0.0)
    max_rotation_speed: float = Field(default=540.0, gt=0.0)
    rotation_acceleration: float = Field(default=1440.0, gt=0.0)
    rotation_deceleration: float = Field(default=1800.0, gt=0.0)
    turn_in_place_threshold: float = Field(default=45.0, ge=1.0, le=180.0)
    pivot_angle_threshold: float = Field(default=135.0, ge=90.0, le=180.0)
    landing_soft_velocity: float = Field(default=4.0, ge=0.0)
    landing_heavy_velocity: float = Field(default=9.0, gt=0.0)
    landing_recovery_seconds: float = Field(default=0.35, ge=0.0)
    apex_velocity_threshold: float = Field(default=0.15, ge=0.0)
    jump_speed: float = Field(default=6.0, gt=0.0)
    gravity: float = Field(default=9.81, gt=0.0)
    max_fall_speed: float = Field(default=50.0, gt=0.0)
    max_walkable_slope: float = Field(default=50.0, gt=0.0, le=89.0)
    fall_damage_start_distance: float = Field(default=5.0, ge=0.0)
    fall_damage_per_meter: float = Field(default=15.0, ge=0.0)
    max_health: float = Field(default=100.0, gt=0.0)
    projectile_speed: float = Field(default=28.0, gt=0.0)
    projectile_radius: float = Field(default=0.08, gt=0.0)
    projectile_range: float = Field(default=60.0, gt=0.0)
    projectile_damage: float = Field(default=34.0, gt=0.0)
    attack_cooldown_seconds: float = Field(default=0.75, gt=0.0)
    jump_cooldown_seconds: float = Field(default=0.25, ge=0.0)
    character_radius: float = Field(default=0.45, gt=0.0)
    character_cylinder_height: float = Field(default=0.9, gt=0.0)
    character_step_height: float = Field(default=0.35, ge=0.0)
    spawn_height: float = Field(default=1.0, gt=0.0)
    log_level: str = "INFO"

    @model_validator(mode="after")
    def validate_speeds(self) -> "Settings":
        if not self.walk_speed <= self.run_speed <= self.sprint_speed:
            raise ValueError("walk_speed, run_speed, and sprint_speed must be ordered")
        if self.landing_soft_velocity >= self.landing_heavy_velocity:
            raise ValueError("landing_soft_velocity must be below landing_heavy_velocity")
        return self

    @property
    def fixed_dt(self) -> float:
        return 1.0 / self.tick_rate


@lru_cache(maxsize=1)
def get_settings() -> Settings:
    return Settings()
