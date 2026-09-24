from collections import deque
from time import monotonic


class RuntimeMetrics:
    def __init__(self) -> None:
        self.tick_durations_ms: deque[float] = deque(maxlen=300)
        self.tick_timestamps: dict[str, deque[float]] = {}
        self.snapshot_send_timestamps: deque[float] = deque(maxlen=2000)
        self.websocket_connections = 0
        self.snapshot_bytes = 0
        self.snapshot_messages = 0
        self.command_queue_sizes: dict[str, int] = {}

    def record_tick(self, room_id: str, duration_ms: float) -> None:
        now = monotonic()
        self.tick_durations_ms.append(duration_ms)
        timestamps = self.tick_timestamps.setdefault(room_id, deque(maxlen=300))
        timestamps.append(now)
        while timestamps and now - timestamps[0] > 5.0:
            timestamps.popleft()

    def record_snapshot(self, payload_bytes: int, recipient_count: int) -> None:
        self.snapshot_bytes = payload_bytes
        now = monotonic()
        self.snapshot_messages += recipient_count
        for _ in range(recipient_count):
            self.snapshot_send_timestamps.append(now)
        while self.snapshot_send_timestamps and now - self.snapshot_send_timestamps[0] > 5.0:
            self.snapshot_send_timestamps.popleft()

    def set_room_command_queue_size(self, room_id: str, size: int) -> None:
        self.command_queue_sizes[room_id] = size

    def remove_room(self, room_id: str) -> None:
        self.command_queue_sizes.pop(room_id, None)
        self.tick_timestamps.pop(room_id, None)

    def snapshot(
        self, room_count: int, player_count: int, tick_rate: int
    ) -> dict[str, int | float]:
        now = monotonic()
        room_tick_rates = [
            sum(1 for timestamp in timestamps if now - timestamp <= 1.0)
            for timestamps in self.tick_timestamps.values()
        ]
        recent_snapshot_sends = sum(
            1 for timestamp in self.snapshot_send_timestamps if now - timestamp <= 1.0
        )
        average_ms = (
            sum(self.tick_durations_ms) / len(self.tick_durations_ms)
            if self.tick_durations_ms
            else 0.0
        )
        return {
            "server_tick_duration_ms_avg": round(average_ms, 3),
            "server_tick_duration_ms_last": round(
                self.tick_durations_ms[-1] if self.tick_durations_ms else 0.0, 3
            ),
            "server_tick_rate_target": tick_rate,
            "server_tick_rate_observed_1s_min_room": min(room_tick_rates, default=0),
            "room_count": room_count,
            "player_count": player_count,
            "websocket_connections": self.websocket_connections,
            "snapshot_size_bytes_last": self.snapshot_bytes,
            "snapshot_send_rate_1s": recent_snapshot_sends,
            "snapshot_messages_total": self.snapshot_messages,
            "command_queue_size": sum(self.command_queue_sizes.values()),
        }
