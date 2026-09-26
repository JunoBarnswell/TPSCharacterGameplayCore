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
from aster_game.room.room import GameRoom


def test_platformer_page_and_modules_are_served() -> None:
    async def scenario() -> None:
        transport = httpx.ASGITransport(app=app_module.app)
        async with httpx.AsyncClient(
            transport=transport, base_url="http://test", trust_env=False
        ) as client:
            page = await client.get("/platformer")
            assert page.status_code == 200
            assert "阿芽的星灯之旅" in page.text
            for path in ("game.mjs", "world.mjs", "levels.mjs"):
                asset = await client.get(f"/web/platformer/{path}")
                assert asset.status_code == 200
                assert "javascript" in asset.headers["content-type"]

    asyncio.run(scenario())


def test_protocol_rejects_out_of_range_and_unknown_fields() -> None:
    for payload in (
        {"type": "input", "sequence": 1, "move_x": 2.0},
        {"type": "input", "sequence": 1, "unexpected": True},
        {"type": "input", "sequence": 1, "client_tick": 1, "yaw": 0.0},
        {"type": "attack", "target_id": 2},
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
            assert metrics_snapshot["snapshot_enqueue_rate_1s"] >= 16 * 57
            for phase in ("movement_solver", "physics", "snapshot"):
                assert metrics_snapshot[f"{phase}_duration_ms_avg"] >= 0
                assert metrics_snapshot[f"{phase}_duration_ms_last"] >= 0
                assert metrics_snapshot[f"{phase}_duration_ms_p95"] >= 0
        finally:
            await manager.close()

    asyncio.run(scenario())


def test_snapshot_profiles_keep_owner_ack_out_of_remote_motion_state() -> None:
    settings = Settings(max_players_per_room=2)
    metrics = RuntimeMetrics()
    room = GameRoom("snapshot-profiles", settings, metrics)
    first = WebSocketSession(None, 64)  # type: ignore[arg-type]
    second = WebSocketSession(None, 64)  # type: ignore[arg-type]
    try:
        first.entity_id = room.add_player(first, "pilot-a", "Pilot A")
        second.entity_id = room.add_player(second, "pilot-b", "Pilot B")
        room._dispatch_snapshot()

        first_message, first_serialized = first._outbound[-1]
        second_message, second_serialized = second._outbound[-1]
        first_wire = json.loads(first_serialized)
        second_wire = json.loads(second_serialized)
        assert first_wire == first_message
        assert second_wire == second_message
        assert first_wire["owner"]["player_id"] == "pilot-a"
        assert second_wire["owner"]["player_id"] == "pilot-b"
        assert [player["player_id"] for player in first_wire["players"]] == ["pilot-b"]
        assert [player["player_id"] for player in second_wire["players"]] == ["pilot-a"]
        owner = first_wire["owner"]
        remote = first_wire["players"][0]
        assert "last_processed_input" in owner
        assert "last_grounded_tick" in owner
        assert 0.0 <= owner["gait_phase"] < 1.0
        assert "jump_available_tick" in owner
        assert {"velocity", "character_yaw", "locomotion_phase", "floor_normal"} <= remote.keys()
        assert {
            "locomotion", "upper_body_action", "additive_reaction",
            "full_body_override", "life_override",
        } <= remote["action_channels"].keys()
        assert remote["action_channels"]["locomotion"]["active"]
        assert remote["action_channels"]["locomotion"]["sequence"] >= 0
        assert "last_processed_input" not in remote
        assert "ground_contact_point" not in remote

        full_world = room.world.snapshot()
        full_payload = json.dumps(
            {"type": "snapshot", **full_world}, separators=(",", ":"), ensure_ascii=False
        ).encode("utf-8")
        max_profile_size = max(
            len(first_serialized.encode("utf-8")), len(second_serialized.encode("utf-8"))
        )
        assert max_profile_size < len(full_payload)
        measured = metrics.snapshot(1, 2, settings.tick_rate)
        assert measured["snapshot_messages_enqueued_total"] == 2
        assert measured["snapshot_size_bytes_last"] == max(
            len(first_serialized.encode("utf-8")), len(second_serialized.encode("utf-8"))
        )
        assert measured["snapshot_size_bytes_avg_last_tick"] > 0
    finally:
        asyncio.run(room.close())


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
                collision_asset = await client.get(
                    f"{http_url}/web/motion/arena-collision.json"
                )
                assert collision_asset.status_code == 200

            async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as obsolete_client:
                await obsolete_client.send(json.dumps({"type": "hello", "protocol_version": 4}))
                unsupported = json.loads(await asyncio.wait_for(obsolete_client.recv(), 2.0))
                assert unsupported["code"] == "UNSUPPORTED_PROTOCOL"

            async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as websocket:
                async with websockets.connect(f"ws://127.0.0.1:{port}/ws") as legacy_socket:
                    await legacy_socket.send(
                        json.dumps({"type": "hello", "protocol_version": 5})
                    )
                    legacy_error = json.loads(await asyncio.wait_for(legacy_socket.recv(), 2.0))
                    assert legacy_error["type"] == "error"
                    assert legacy_error["code"] == "UNSUPPORTED_PROTOCOL"
                    assert "version is 7" in legacy_error["message"]

                await websocket.send(json.dumps({"type": "hello", "protocol_version": 7}))
                welcome = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                assert welcome["type"] == "welcome"
                assert welcome["protocol_version"] == 7
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
                    "ground_directional_friction": settings.ground_directional_friction,
                    "turning_deceleration": settings.turning_deceleration,
                    "pivot_braking_multiplier": settings.pivot_braking_multiplier,
                    "air_acceleration": settings.air_acceleration,
                    "air_max_speed": settings.air_max_speed,
                    "max_rotation_speed": settings.max_rotation_speed,
                    "rotation_acceleration": settings.rotation_acceleration,
                    "rotation_deceleration": settings.rotation_deceleration,
                    "walk_acceleration_curve": [
                        list(point) for point in settings.walk_acceleration_curve
                    ],
                    "run_acceleration_curve": [
                        list(point) for point in settings.run_acceleration_curve
                    ],
                    "sprint_acceleration_curve": [
                        list(point) for point in settings.sprint_acceleration_curve
                    ],
                    "braking_curve": [list(point) for point in settings.braking_curve],
                    "turn_speed_curve": [list(point) for point in settings.turn_speed_curve],
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
                    "max_walkable_slope": settings.max_walkable_slope,
                    "ground_probe_radius": settings.ground_probe_radius,
                    "ground_probe_depth": settings.ground_probe_depth,
                    "ground_probe_start_offset": settings.ground_probe_start_offset,
                    "ground_snap_distance": settings.ground_snap_distance,
                    "ground_grace_distance": settings.ground_grace_distance,
                    "ground_grace_ticks": settings.ground_grace_ticks,
                    "character_step_height": settings.character_step_height,
                    "character_radius": settings.character_radius,
                    "character_cylinder_height": settings.character_cylinder_height,
                }
                assert welcome["collision_world"]["version"] == 1
                assert welcome["collision_world"]["planes"][0]["name"] == "arena-floor"
                assert any(
                    box["name"] == "cover-center" for box in welcome["collision_world"]["boxes"]
                )
                assert any(
                    box["name"] == "upper-platform-step-1"
                    for box in welcome["collision_world"]["boxes"]
                )
                assert welcome["collision_world"]["ramps"][0]["name"] == "walkable-ramp"

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
                            "requested_gait": "sprint",
                            "view_yaw": 0.0,
                            "view_pitch": 0.0,
                            "rotation_mode": "orient_to_movement",
                        }
                    )
                )
                for _ in range(60):
                    message = json.loads(await asyncio.wait_for(websocket.recv(), 2.0))
                    if message["type"] == "snapshot":
                        player = message["owner"]
                        assert player["player_id"] == joined["player_id"]
                        if player["last_processed_input"] == 7:
                            assert player["position"][2] > -12.0
                            assert player["movement_mode"] == "grounded"
                            assert "locomotion_phase" in player
                            assert all(
                                "last_processed_input" not in remote
                                for remote in message["players"]
                            )
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
                for phase in ("movement_solver", "physics", "snapshot"):
                    assert metrics[f"{phase}_duration_ms_avg"] >= 0
                    assert metrics[f"{phase}_duration_ms_last"] >= 0
                    assert metrics[f"{phase}_duration_ms_p95"] >= 0
                assert metrics["server_tick_duration_ms_p95"] >= 0
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
