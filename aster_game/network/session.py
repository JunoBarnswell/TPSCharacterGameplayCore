import asyncio
from collections import deque
from typing import Any
from uuid import uuid4

from fastapi import WebSocket


class WebSocketSession:
    def __init__(self, websocket: WebSocket, outbound_capacity: int) -> None:
        self.websocket = websocket
        self.session_id = uuid4().hex
        self.player_id: str | None = None
        self.room_id: str | None = None
        self.entity_id: int | None = None
        self.protocol_version: int | None = None
        self.last_received_input_sequence = -1
        self._outbound_capacity = outbound_capacity
        self._outbound: deque[tuple[dict[str, Any], str | None]] = deque()
        self._outbound_ready = asyncio.Event()
        self._close_request: tuple[int, str] | None = None
        self.closed = False

    def enqueue(self, message: dict[str, Any], *, serialized: str | None = None) -> bool:
        if self.closed or self._close_request is not None:
            return False
        is_snapshot = message.get("type") == "snapshot"
        if serialized is not None and not is_snapshot:
            raise ValueError("pre-serialized outbound payloads are reserved for snapshots")
        if len(self._outbound) >= self._outbound_capacity:
            if is_snapshot:
                for index, (queued, _) in enumerate(self._outbound):
                    if queued.get("type") == "snapshot":
                        del self._outbound[index]
                        break
                else:
                    return False
            else:
                for index, (queued, _) in enumerate(self._outbound):
                    if queued.get("type") == "snapshot":
                        del self._outbound[index]
                        break
                else:
                    self.request_close(1013, "outbound queue is full")
                    return False
        self._outbound.append((message, serialized))
        self._outbound_ready.set()
        return True

    def request_close(self, code: int, reason: str) -> None:
        if self._close_request is None:
            self._close_request = (code, reason)
        self._outbound_ready.set()

    async def send_loop(self) -> None:
        try:
            while True:
                await self._outbound_ready.wait()
                while self._outbound:
                    message, serialized = self._outbound.popleft()
                    if serialized is None:
                        await self.websocket.send_json(message)
                    else:
                        await self.websocket.send_text(serialized)
                self._outbound_ready.clear()
                if self._close_request is not None:
                    code, reason = self._close_request
                    await self.websocket.close(code=code, reason=reason)
                    return
        except Exception:
            self.closed = True
            raise
        finally:
            self.closed = True
