import mimetypes
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from aster_game.app.config import Settings, get_settings
from aster_game.infrastructure.logging import configure_logging
from aster_game.infrastructure.metrics import RuntimeMetrics
from aster_game.network.gateway import handle_websocket
from aster_game.room.manager import RoomManager


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncIterator[None]:
    settings = get_settings()
    configure_logging(settings.log_level)
    metrics = RuntimeMetrics()
    rooms = RoomManager(settings, metrics)
    app.state.settings = settings
    app.state.metrics = metrics
    app.state.rooms = rooms
    try:
        yield
    finally:
        await rooms.close()


app = FastAPI(title="Aster Gameplay Server", version="0.1.0", lifespan=lifespan)
TEST_PAGE = Path(__file__).parents[1] / "web" / "test_client.html"
WEB_ASSETS = Path(__file__).parents[1] / "web"
mimetypes.add_type("text/javascript", ".mjs")
app.mount("/web", StaticFiles(directory=WEB_ASSETS), name="web-assets")


@app.get("/", include_in_schema=False)
@app.get("/test", include_in_schema=False)
async def gameplay_test_page() -> FileResponse:
    return FileResponse(TEST_PAGE, media_type="text/html")


@app.get("/healthz")
async def health() -> dict[str, str | int]:
    rooms: RoomManager | None = getattr(app.state, "rooms", None)
    return {
        "status": "ok",
        "room_count": len(rooms.rooms) if rooms is not None else 0,
    }


@app.get("/metrics")
async def metrics_endpoint() -> dict[str, int | float]:
    settings: Settings = getattr(app.state, "settings", get_settings())
    metrics: RuntimeMetrics | None = getattr(app.state, "metrics", None)
    rooms: RoomManager | None = getattr(app.state, "rooms", None)
    if metrics is None or rooms is None:
        return {
            "server_tick_duration_ms_avg": 0.0,
            "server_tick_duration_ms_last": 0.0,
            "server_tick_rate_target": settings.tick_rate,
            "server_tick_rate_observed_1s_min_room": 0,
            "room_count": 0,
            "player_count": 0,
            "websocket_connections": 0,
            "snapshot_size_bytes_last": 0,
            "snapshot_send_rate_1s": 0,
            "snapshot_messages_total": 0,
            "command_queue_size": 0,
        }
    return metrics.snapshot(len(rooms.rooms), rooms.player_count, settings.tick_rate)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    settings: Settings = websocket.app.state.settings
    metrics: RuntimeMetrics = websocket.app.state.metrics
    rooms: RoomManager = websocket.app.state.rooms
    await handle_websocket(websocket, settings, metrics, rooms)
