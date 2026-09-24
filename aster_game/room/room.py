import asyncio
import json
import logging
from time import perf_counter

from aster_game.app.config import Settings
from aster_game.game.events import GameplayEvent
from aster_game.game.world import GameWorld
from aster_game.infrastructure.metrics import RuntimeMetrics
from aster_game.network.messages import (
    OwnerPlayerSnapshot,
    RemotePlayerSnapshot,
    SnapshotMessage,
)
from aster_game.network.session import WebSocketSession

logger = logging.getLogger(__name__)


class GameRoom:
    def __init__(
        self,
        room_id: str,
        settings: Settings,
        metrics: RuntimeMetrics,
    ) -> None:
        self.room_id = room_id
        self.settings = settings
        self.metrics = metrics
        self.world = GameWorld(room_id, settings, metrics)
        self.sessions: dict[str, WebSocketSession] = {}
        self._task: asyncio.Task[None] | None = None
        self._stopping = False
        self.failed = False

    @property
    def player_count(self) -> int:
        return len(self.sessions)

    @property
    def is_full(self) -> bool:
        return self.player_count >= self.settings.max_players_per_room

    def start(self) -> None:
        if self._task is not None:
            raise RuntimeError("room loop is already running")
        self._task = asyncio.create_task(self._run(), name=f"game-room-{self.room_id}")

    def add_player(
        self,
        session: WebSocketSession,
        player_id: str,
        player_name: str,
    ) -> int:
        if self.is_full:
            raise OverflowError("room is full")
        character = self.world.add_player(player_id, player_name)
        self.sessions[player_id] = session
        return character.entity_id

    def remove_player(self, player_id: str) -> None:
        self.sessions.pop(player_id, None)
        self.world.remove_player(player_id)

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        interval = self.settings.fixed_dt
        deadline = loop.time()
        try:
            while not self._stopping:
                started = perf_counter()
                events = self.world.tick(interval)
                self._dispatch_events(events)
                if self.world.tick_id % self.settings.snapshot_interval_ticks == 0:
                    self._dispatch_snapshot()
                duration_ms = (perf_counter() - started) * 1000.0
                self.metrics.record_tick(self.room_id, duration_ms)
                if duration_ms >= interval * 1000.0:
                    logger.warning(
                        "fixed tick exceeded its budget",
                        extra={
                            "event": "TickSlow",
                            "room_id": self.room_id,
                            "tick": self.world.tick_id,
                            "duration_ms": round(duration_ms, 3),
                        },
                    )

                deadline += interval
                delay = deadline - loop.time()
                if delay > 0:
                    await asyncio.sleep(delay)
                else:
                    await asyncio.sleep(0)
        except asyncio.CancelledError:
            raise
        except Exception:
            self.failed = True
            logger.exception(
                "room simulation stopped after an unhandled error",
                extra={"event": "RoomSimulationFailed", "room_id": self.room_id},
            )
            for session in self.sessions.values():
                session.enqueue(
                    {
                        "type": "error",
                        "code": "ROOM_SIMULATION_FAILED",
                        "message": "Room simulation stopped",
                    }
                )
                session.request_close(1011, "room simulation failed")
            self._stopping = True

    def _dispatch_events(self, events: list[GameplayEvent]) -> None:
        for event in events:
            message = {"type": event.type, "tick": event.tick, **event.data}
            for session in tuple(self.sessions.values()):
                session.enqueue(message)
            if event.type in {
                "character_spawn",
                "character_left",
                "hit",
                "damage",
                "fall_impact",
                "death",
                "respawn",
            }:
                logger.info(
                    "gameplay event",
                    extra={
                        "room_id": self.room_id,
                        "tick": event.tick,
                        **self._event_log_fields(event),
                    },
                )

    @staticmethod
    def _event_log_fields(event: GameplayEvent) -> dict[str, object]:
        logged_fields = {
            "entity_id",
            "source_entity_id",
            "target_entity_id",
            "damage_type",
        }
        fields = {key: value for key, value in event.data.items() if key in logged_fields}
        fields["event"] = {
            "character_spawn": "CharacterSpawn",
            "character_left": "PlayerLeft",
            "hit": "Hit",
            "damage": "Damage",
            "fall_impact": "FallImpact",
            "death": "CharacterDeath",
            "respawn": "CharacterRespawn",
        }.get(event.type, event.type)
        return fields

    def _dispatch_snapshot(self) -> None:
        started = perf_counter()
        world_snapshot = self.world.snapshot()
        player_snapshots = {player["entity_id"]: player for player in world_snapshot["players"]}
        remote_models = {
            entity_id: RemotePlayerSnapshot.model_validate(
                {name: player[name] for name in RemotePlayerSnapshot.model_fields}
            )
            for entity_id, player in player_snapshots.items()
        }
        payload_sizes: list[int] = []
        for session in tuple(self.sessions.values()):
            if session.entity_id not in player_snapshots:
                raise RuntimeError("room session does not own a character in its world snapshot")
            owner = OwnerPlayerSnapshot.model_validate(player_snapshots[session.entity_id])
            message = SnapshotMessage(
                tick=world_snapshot["tick"],
                owner=owner,
                players=[
                    remote
                    for entity_id, remote in remote_models.items()
                    if entity_id != session.entity_id
                ],
                projectiles=world_snapshot["projectiles"],
            ).model_dump(mode="json")
            serialized = json.dumps(message, separators=(",", ":"), ensure_ascii=False)
            if session.enqueue(message, serialized=serialized):
                payload_sizes.append(len(serialized.encode("utf-8")))
        self.metrics.record_snapshot(payload_sizes)
        self.metrics.record_phase("snapshot", (perf_counter() - started) * 1000.0)

    async def close(self) -> None:
        self._stopping = True
        for session in tuple(self.sessions.values()):
            session.request_close(1001, "room is shutting down")
        if self._task is not None and self._task is not asyncio.current_task():
            self._task.cancel()
            await asyncio.gather(self._task, return_exceptions=True)
        self.world.close()
        logger.info("room destroyed", extra={"event": "RoomDestroyed", "room_id": self.room_id})
