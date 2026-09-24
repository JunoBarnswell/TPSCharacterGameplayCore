import asyncio
import logging
from contextlib import suppress

from fastapi import WebSocket, WebSocketDisconnect
from pydantic import ValidationError

from aster_game.app.config import Settings
from aster_game.game.components import InputCommand
from aster_game.infrastructure.metrics import RuntimeMetrics
from aster_game.network.messages import (
    CLIENT_MESSAGE_ADAPTER,
    AttackMessage,
    HelloMessage,
    InputMessage,
    JoinedMessage,
    JoinGameMessage,
    MovementTuning,
    PingMessage,
    PongMessage,
    RespawnMessage,
    WelcomeMessage,
)
from aster_game.network.session import WebSocketSession
from aster_game.room.manager import (
    RoomFullError,
    RoomManager,
    RoomNotFoundError,
    RoomUnavailableError,
    ServerCapacityError,
)

logger = logging.getLogger(__name__)


class HeartbeatTimeout(Exception):
    pass


class WriterStopped(Exception):
    pass


class InvalidProtocolMessage(Exception):
    pass


async def _receive_json(
    session: WebSocketSession,
    writer_task: asyncio.Task[None],
    timeout_seconds: float,
) -> object:
    receive_task = asyncio.create_task(session.websocket.receive_json())
    done, _ = await asyncio.wait(
        {receive_task, writer_task},
        timeout=timeout_seconds,
        return_when=asyncio.FIRST_COMPLETED,
    )
    if receive_task in done:
        return receive_task.result()
    receive_task.cancel()
    await asyncio.gather(receive_task, return_exceptions=True)
    if writer_task in done:
        error = writer_task.exception()
        if error is not None:
            raise error
        raise WriterStopped
    raise HeartbeatTimeout


def _parse_message(raw: object) -> object:
    try:
        return CLIENT_MESSAGE_ADAPTER.validate_python(raw)
    except ValidationError as exc:
        raise InvalidProtocolMessage("Invalid message payload") from exc


def _enqueue_error(session: WebSocketSession, code: str, message: str) -> None:
    session.enqueue({"type": "error", "code": code, "message": message})


async def handle_websocket(
    websocket: WebSocket,
    settings: Settings,
    metrics: RuntimeMetrics,
    rooms: RoomManager,
) -> None:
    await websocket.accept()
    metrics.websocket_connections += 1
    session = WebSocketSession(websocket, settings.max_outbound_messages)
    logger.info(
        "websocket connected",
        extra={"event": "WebSocketConnected", "session_id": session.session_id},
    )
    writer_task = asyncio.create_task(session.send_loop(), name=f"ws-writer-{session.session_id}")
    try:
        try:
            raw_hello = await _receive_json(
                session, writer_task, settings.heartbeat_timeout_seconds
            )
        except HeartbeatTimeout:
            _enqueue_error(session, "HELLO_TIMEOUT", "Send hello before the heartbeat deadline")
            session.request_close(1002, "hello timeout")
            return
        hello = _parse_message(raw_hello)
        if not isinstance(hello, HelloMessage):
            _enqueue_error(session, "HELLO_REQUIRED", "The first message must be hello")
            session.request_close(1002, "hello required")
            return
        if hello.protocol_version != 2:
            _enqueue_error(session, "UNSUPPORTED_PROTOCOL", "Supported protocol version is 2")
            session.request_close(1002, "unsupported protocol")
            return
        session.protocol_version = hello.protocol_version
        session.enqueue(
            WelcomeMessage(
                session_id=session.session_id,
                tick_rate=settings.tick_rate,
                snapshot_interval_ticks=settings.snapshot_interval_ticks,
                movement_tuning=MovementTuning(
                    walk_speed=settings.walk_speed,
                    run_speed=settings.run_speed,
                    sprint_speed=settings.sprint_speed,
                    air_control=settings.air_control,
                    ground_acceleration=settings.ground_acceleration,
                    braking_deceleration=settings.braking_deceleration,
                    ground_friction=settings.ground_friction,
                    air_acceleration=settings.air_acceleration,
                    air_max_speed=settings.air_max_speed,
                    max_rotation_speed=settings.max_rotation_speed,
                    rotation_acceleration=settings.rotation_acceleration,
                    rotation_deceleration=settings.rotation_deceleration,
                    turn_in_place_threshold=settings.turn_in_place_threshold,
                    pivot_angle_threshold=settings.pivot_angle_threshold,
                    jump_speed=settings.jump_speed,
                    gravity=settings.gravity,
                    max_fall_speed=settings.max_fall_speed,
                    apex_velocity_threshold=settings.apex_velocity_threshold,
                    jump_cooldown_seconds=settings.jump_cooldown_seconds,
                    landing_soft_velocity=settings.landing_soft_velocity,
                    landing_heavy_velocity=settings.landing_heavy_velocity,
                    landing_recovery_seconds=settings.landing_recovery_seconds,
                ),
            ).model_dump(mode="json")
        )

        while True:
            try:
                raw_message = await _receive_json(
                    session, writer_task, settings.heartbeat_timeout_seconds
                )
            except HeartbeatTimeout:
                _enqueue_error(session, "HEARTBEAT_TIMEOUT", "No message received before timeout")
                session.request_close(1001, "heartbeat timeout")
                return
            message = _parse_message(raw_message)

            if isinstance(message, HelloMessage):
                _enqueue_error(session, "HELLO_ALREADY_ACCEPTED", "Hello may only be sent once")
                continue
            if isinstance(message, PingMessage):
                session.enqueue(PongMessage(nonce=message.nonce).model_dump(mode="json"))
                continue
            if session.player_id is None:
                if not isinstance(message, JoinGameMessage):
                    _enqueue_error(
                        session,
                        "JOIN_REQUIRED",
                        "Join a room before sending gameplay commands",
                    )
                    continue
                try:
                    room_id, player_id, entity_id, tick = await rooms.join(
                        session,
                        message.player_name,
                        message.room_id,
                    )
                except RoomNotFoundError:
                    _enqueue_error(session, "ROOM_NOT_FOUND", "The requested room does not exist")
                    continue
                except RoomFullError as exc:
                    _enqueue_error(session, "ROOM_FULL", str(exc))
                    continue
                except RoomUnavailableError as exc:
                    _enqueue_error(session, "ROOM_UNAVAILABLE", str(exc))
                    continue
                except ServerCapacityError as exc:
                    _enqueue_error(session, "SERVER_CAPACITY", str(exc))
                    continue
                session.enqueue(
                    JoinedMessage(
                        room_id=room_id,
                        player_id=player_id,
                        entity_id=entity_id,
                        tick=tick,
                    ).model_dump(mode="json")
                )
                continue

            room = rooms.get_room(session.room_id)
            if room is None or session.entity_id is None:
                _enqueue_error(
                    session,
                    "SESSION_NOT_IN_ROOM",
                    "The room session is no longer active",
                )
                session.request_close(1011, "room session missing")
                return

            if isinstance(message, InputMessage):
                if message.sequence <= session.last_received_input_sequence:
                    _enqueue_error(session, "OUT_OF_ORDER_INPUT", "Input sequence must increase")
                    continue
                command = InputCommand(
                    sequence=message.sequence,
                    client_tick=message.client_tick,
                    move_x=message.move_x,
                    move_z=message.move_z,
                    jump=message.jump,
                    sprint=message.sprint,
                    view_yaw=message.view_yaw,
                    view_pitch=message.view_pitch,
                    rotation_mode=message.rotation_mode.value,
                )
                if not room.world.queue_input(session.entity_id, command):
                    _enqueue_error(
                        session,
                        "INPUT_REJECTED",
                        "Player is not accepting input",
                    )
                    continue
                session.last_received_input_sequence = message.sequence
            elif isinstance(message, AttackMessage):
                if not room.world.queue_attack(session.entity_id):
                    _enqueue_error(session, "COMMAND_QUEUE_FULL", "Too many pending attacks")
            elif isinstance(message, RespawnMessage):
                if not room.world.queue_respawn(session.entity_id):
                    _enqueue_error(session, "RESPAWN_REJECTED", "Player is not available")
            else:
                _enqueue_error(
                    session,
                    "MESSAGE_NOT_ALLOWED",
                    "Message is not valid in this session state",
                )
    except WebSocketDisconnect:
        pass
    except (InvalidProtocolMessage, ValueError):
        _enqueue_error(session, "INVALID_MESSAGE", "Message does not match the protocol")
    except WriterStopped:
        pass
    except Exception:
        logger.exception(
            "websocket session failed",
            extra={"event": "WebSocketSessionFailed", "player_id": session.player_id},
        )
        _enqueue_error(session, "INTERNAL_ERROR", "Session ended after an internal error")
    finally:
        disconnected_player_id = session.player_id
        await rooms.leave(session)
        session.request_close(1000, "connection closed")
        try:
            await asyncio.wait_for(writer_task, timeout=1.0)
        except TimeoutError:
            writer_task.cancel()
            await asyncio.gather(writer_task, return_exceptions=True)
        except Exception:
            with suppress(Exception):
                writer_task.result()
        metrics.websocket_connections = max(0, metrics.websocket_connections - 1)
        logger.info(
            "websocket disconnected",
            extra={"event": "WebSocketDisconnected", "player_id": disconnected_player_id},
        )
