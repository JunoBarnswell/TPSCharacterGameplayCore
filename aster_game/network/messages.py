from typing import Annotated, Literal, TypeAlias

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, field_validator


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
    move_x: float = Field(default=0.0, ge=-1.0, le=1.0, allow_inf_nan=False)
    move_z: float = Field(default=0.0, ge=-1.0, le=1.0, allow_inf_nan=False)
    jump: bool = False
    sprint: bool = False
    yaw: float = Field(default=0.0, ge=-180.0, le=180.0, allow_inf_nan=False)


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


class WelcomeMessage(WireModel):
    type: Literal["welcome"] = "welcome"
    session_id: str
    protocol_version: int = 1
    tick_rate: int


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
    yaw: float
    state: str
    health: float
    max_health: float
    alive: bool
    last_processed_input: int


class ProjectileSnapshot(WireModel):
    projectile_id: int
    owner_entity_id: int
    position: tuple[float, float, float]


class SnapshotMessage(WireModel):
    type: Literal["snapshot"] = "snapshot"
    tick: int
    players: list[PlayerSnapshot]
    projectiles: list[ProjectileSnapshot]
