import asyncio
import logging
from uuid import uuid4

from aster_game.app.config import Settings
from aster_game.infrastructure.metrics import RuntimeMetrics
from aster_game.network.session import WebSocketSession
from aster_game.room.room import GameRoom

logger = logging.getLogger(__name__)


class RoomNotFoundError(Exception):
    pass


class ServerCapacityError(Exception):
    pass


class RoomFullError(ServerCapacityError):
    pass


class RoomUnavailableError(ServerCapacityError):
    pass


class RoomManager:
    def __init__(self, settings: Settings, metrics: RuntimeMetrics) -> None:
        self.settings = settings
        self.metrics = metrics
        self.rooms: dict[str, GameRoom] = {}
        self._lock = asyncio.Lock()
        self._closed = False

    @property
    def player_count(self) -> int:
        return sum(room.player_count for room in self.rooms.values())

    def get_room(self, room_id: str | None) -> GameRoom | None:
        return self.rooms.get(room_id) if room_id is not None else None

    async def join(
        self,
        session: WebSocketSession,
        player_name: str,
        requested_room_id: str | None,
    ) -> tuple[str, str, int, int]:
        async with self._lock:
            if self._closed:
                raise ServerCapacityError("server is shutting down")

            created = False
            if requested_room_id is not None:
                room = self.rooms.get(requested_room_id)
                if room is None:
                    raise RoomNotFoundError(requested_room_id)
                if room.failed:
                    raise RoomUnavailableError("room simulation is unavailable")
                if room.is_full:
                    raise RoomFullError("room is full")
            else:
                candidates = sorted(
                    (room for room in self.rooms.values() if not room.is_full and not room.failed),
                    key=lambda room: (room.player_count, room.room_id),
                )
                if candidates:
                    room = candidates[0]
                else:
                    if len(self.rooms) >= self.settings.max_rooms:
                        raise ServerCapacityError("server room capacity reached")
                    room_id = f"room-{uuid4().hex[:12]}"
                    room = GameRoom(room_id, self.settings, self.metrics)
                    self.rooms[room_id] = room
                    room.start()
                    created = True
                    logger.info("room created", extra={"event": "RoomCreated", "room_id": room_id})

            player_id = uuid4().hex
            try:
                entity_id = room.add_player(session, player_id, player_name)
            except Exception:
                if created:
                    self.rooms.pop(room.room_id, None)
                    await room.close()
                raise

            session.player_id = player_id
            session.room_id = room.room_id
            session.entity_id = entity_id
            logger.info(
                "player joined room",
                extra={
                    "event": "PlayerJoined",
                    "room_id": room.room_id,
                    "player_id": player_id,
                    "entity_id": entity_id,
                },
            )
            return room.room_id, player_id, entity_id, room.world.tick_id

    async def leave(self, session: WebSocketSession) -> None:
        if session.player_id is None or session.room_id is None:
            return
        room_to_close: GameRoom | None = None
        async with self._lock:
            room = self.rooms.get(session.room_id)
            if room is not None:
                room.remove_player(session.player_id)
                logger.info(
                    "player left room",
                    extra={
                        "event": "PlayerLeft",
                        "room_id": room.room_id,
                        "player_id": session.player_id,
                        "entity_id": session.entity_id,
                    },
                )
                if room.player_count == 0:
                    self.rooms.pop(room.room_id, None)
                    room_to_close = room
        session.player_id = None
        session.room_id = None
        session.entity_id = None
        if room_to_close is not None:
            await room_to_close.close()

    async def close(self) -> None:
        async with self._lock:
            self._closed = True
            rooms = list(self.rooms.values())
            self.rooms.clear()
        await asyncio.gather(*(room.close() for room in rooms), return_exceptions=False)
