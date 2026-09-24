from __future__ import annotations

import asyncio
import json
from time import perf_counter

import httpx
import uvicorn
import websockets
from pydantic import ValidationError

from aster_game.app import main as app_module
from aster_game.app.config import Settings
from aster_game.infrastructure.metrics import RuntimeMetrics
from aster_game.network.messages import CLIENT_MESSAGE_ADAPTER, InputMessage
from aster_game.network.session import WebSocketSession
from aster_game.room.manager import RoomManager


def test_protocol_rejects_out_of_range_and_unknown_fields() -> None:
    for payload in (
        {"type": "input", "sequence": 1, "move_x": 2.0},
        {"type": "input", "sequence": 1, "unexpected": True},
        {"type": "input", "sequence": 1, "client_tick": 1, "yaw": 0.0},
    ):
        try:
            CLIENT_MESSAGE_ADAPTER.validate_python(payload)
        except ValueError:
            continue
        raise AssertionError("invalid client payload was accepted")

    parsed = CLIENT_MESSAGE_ADAPTER.validate_python(
        {"type": "input", "sequence": 1, "client_tick": 1, "move_z": 1.0}
    )
    assert isinstance(parsed, InputMessage)


def test_server_tick_rate_requires_at_least_60_hz() -> None:
    defaults = Settings()
    assert defaults.tick_rate == 60
    assert defaults.snapshot_interval_ticks == 3
    try:
        Settings(tick_rate=59)
    except ValidationError:
        return
    raise AssertionError("server accepted a tick rate below 60 Hz")


def test_room_manager_creates_independent_worlds_at_capacity() -> None:
    async def scenario() -> None:
        settings = Settings(max_players_per_room=2, max_rooms=2)
        manager = RoomManager(settings, RuntimeMetrics())
        sessions = [WebSocketSession(None, 64) for _ in range(3)]  # type: ignore[arg-type]
        try:
            joined = [
                await manager.join(session, f"Player {index}", None)
                for index, session in enumerate(sessions)
            ]
            assert joined[0][0] == joined[1][0]
            assert joined[2][0] != joined[0][0]
            first = manager.get_room(joined[0][0])
            second = manager.get_room(joined[2][0])
            assert first is not None and second is not None
            assert first.world is not second.world
            assert first.world.physics is not second.world.physics
        finally:
            await manager.close()

    asyncio.run(scenario())


def test_room_loop_runs_near_configured_fixed_tick_rate() -> None:
    async def scenario() -> None:
        settings = Settings(
            tick_rate=60,
            snapshot_interval_ticks=1,
            max_players_per_room=16,
        )
        metrics = RuntimeMetrics()
        manager = RoomManager(settings, metrics)
        sessions = [WebSocketSession(None, 64) for _ in range(settings.max_players_per_room)]
        try:
            joined = [
                await manager.join(session, f"Runner {index}", None)
                for index, session in enumerate(sessions)
            ]
            room_id = joined[0][0]
            assert all(joined_room_id == room_id for joined_room_id, _, _, _ in joined)
            room = manager.get_room(room_id)
            assert room is not None

            started = perf_counter()
            await asyncio.sleep(1.1)
            elapsed = perf_counter() - started
            measured_rate = room.world.tick_id / elapsed

            assert 57.0 <= measured_rate <= 64.0
            metrics_snapshot = metrics.snapshot(1, len(sessions), settings.tick_rate)
            assert metrics_snapshot["server_tick_rate_observed_1s_min_room"] >= 57
            assert metrics_snapshot["snapshot_send_rate_1s"] >= 16 * 57
        finally:
            await manager.close()

    asyncio.run(scenario())


def test_websocket_handshake_commands_snapshot_and_metrics(monkeypatch) -> None:
    settings = Settings(snapshot_interval_ticks=1, heartbeat_timeout_seconds=5.0)
    monkeypatch.setattr(app_module, "get_settings", lambda: settings)

    async def scenario() -> None:
        server = uvicorn.Server(
            uvicorn.Config(
                app_module.app,
                host="127.0.0.1",
                port=0,
                log_config=None,
                access_log=False,
            )
        )
        server_task = asyncio.create_task(server.serve())
        try:
            deadline = asyncio.get_running_loop().time() + 5.0
            while not server.started:
                if server_task.done():
                    await server_task
                    raise AssertionError("Uvicorn stopped before startup")
                if asyncio.get_running_loop().time() >= deadline:
                    raise AssertionError("Uvicorn did not start within five seconds")
                await asyncio.sleep(0.01)

            assert server.servers
            port = server.servers[0].sockets[0].getsockname()[1]
            http_url = f"http://127.0.0.1:{port}"
            async with httpx.AsyncClient() as client:
                page_response = await client.get(f"{http_url}/")
                assert page_response.status_code == 200
                assert "Aster Character Motion Lab" in page_response.text
                motion_asset = await client.get(f"{http_url}/web/motion/movement-solver.mjs")
                assert motion_asset.status_code == 200
                assert "javascript" in motion_asset.headers["content-type"]
                assert "solveHorizontalVelocity" in motion_asset.text

            async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as websocket:
                await websocket.send(json.dumps({"type": "hello", "protocol_version": 2}))
                welcome = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                assert welcome["type"] == "welcome"
                assert welcome["protocol_version"] == 2
                assert welcome["tick_rate"] == settings.tick_rate
                assert welcome["snapshot_interval_ticks"] == settings.snapshot_interval_ticks
                assert welcome["movement_tuning"] == {
                    "walk_speed": settings.walk_speed,
                    "run_speed": settings.run_speed,
                    "sprint_speed": settings.sprint_speed,
                    "air_control": settings.air_control,
                    "ground_acceleration": settings.ground_acceleration,
                    "braking_deceleration": settings.braking_deceleration,
                    "ground_friction": settings.ground_friction,
                    "air_acceleration": settings.air_acceleration,
                    "air_max_speed": settings.air_max_speed,
                    "max_rotation_speed": settings.max_rotation_speed,
                    "rotation_acceleration": settings.rotation_acceleration,
                    "rotation_deceleration": settings.rotation_deceleration,
                    "turn_in_place_threshold": settings.turn_in_place_threshold,
                    "pivot_angle_threshold": settings.pivot_angle_threshold,
                    "jump_speed": settings.jump_speed,
                    "gravity": settings.gravity,
                    "max_fall_speed": settings.max_fall_speed,
                    "apex_velocity_threshold": settings.apex_velocity_threshold,
                    "jump_cooldown_seconds": settings.jump_cooldown_seconds,
                    "landing_soft_velocity": settings.landing_soft_velocity,
                    "landing_heavy_velocity": settings.landing_heavy_velocity,
                    "landing_recovery_seconds": settings.landing_recovery_seconds,
                }

                await websocket.send(json.dumps({"type": "join_game", "player_name": "  Pilot  "}))
                joined = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                assert joined["type"] == "joined"

                await websocket.send(json.dumps({"type": "ping", "nonce": "check"}))
                for _ in range(20):
                    if json.loads(await asyncio.wait_for(websocket.recv(), 2.0))["type"] == "pong":
                        break
                else:
                    raise AssertionError("server did not return pong")

                await websocket.send(
                    json.dumps(
                        {
                            "type": "input",
                            "sequence": 7,
                            "client_tick": 7,
                            "move_x": 0.0,
                            "move_z": 1.0,
                            "jump": False,
                            "sprint": True,
                            "view_yaw": 0.0,
                            "view_pitch": 0.0,
                            "rotation_mode": "orient_to_movement",
                        }
                    )
                )
                for _ in range(60):
                    message = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                    if message["type"] == "snapshot":
                        player = next(
                            player
                            for player in message["players"]
                            if player["player_id"] == joined["player_id"]
                        )
                        if player["last_processed_input"] == 7:
                            assert player["position"][2] > -12.0
                            assert player["movement_mode"] == "grounded"
                            assert "locomotion_phase" in player
                            break
                else:
                    raise AssertionError("server did not acknowledge input in a snapshot")

                async with httpx.AsyncClient() as client:
                    metrics_response = await client.get(f"{http_url}/metrics")
                    health_response = await client.get(f"{http_url}/healthz")
                metrics = metrics_response.json()
                assert metrics["room_count"] == 1
                assert metrics["player_count"] == 1
                assert metrics["websocket_connections"] == 1
                assert health_response.json()["status"] == "ok"
        finally:
            server.should_exit = True
            try:
                await asyncio.wait_for(server_task, 5.0)
            except TimeoutError:
                server.force_exit = True
                server_task.cancel()
                await asyncio.gather(server_task, return_exceptions=True)

    asyncio.run(scenario())
