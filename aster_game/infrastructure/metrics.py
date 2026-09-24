from collections import deque
from math import ceil
from time import monotonic


class RuntimeMetrics:
    def __init__(self) -> None:
        self.tick_durations_ms: deque[float] = deque(maxlen=300)
        self.phase_durations_ms: dict[str, deque[float]] = {
            "movement_solver": deque(maxlen=300),
            "physics": deque(maxlen=300),
            "snapshot": deque(maxlen=300),
        }
        self.tick_timestamps: dict[str, deque[float]] = {}
        self.snapshot_send_samples: deque[tuple[float, int]] = deque(maxlen=20_000)
        self.websocket_connections = 0
        self.snapshot_size_bytes_max_last_tick = 0
        self.snapshot_size_bytes_avg_last_tick = 0.0
        self.snapshot_bytes_enqueued_total = 0
        self.snapshot_messages = 0
        self.command_queue_sizes: dict[str, int] = {}

    def record_tick(self, room_id: str, duration_ms: float) -> None:
        now = monotonic()
        self.tick_durations_ms.append(duration_ms)
        timestamps = self.tick_timestamps.setdefault(room_id, deque(maxlen=300))
        timestamps.append(now)
        while timestamps and now - timestamps[0] > 5.0:
            timestamps.popleft()

    def record_phase(self, phase: str, duration_ms: float) -> None:
        if phase not in self.phase_durations_ms:
            raise ValueError(f"unknown runtime metric phase: {phase}")
        if duration_ms < 0:
            raise ValueError("runtime phase duration must be non-negative")
        self.phase_durations_ms[phase].append(duration_ms)

    @staticmethod
    def _p95(durations: deque[float]) -> float:
        if not durations:
            return 0.0
        ordered = sorted(durations)
        return ordered[max(0, ceil(0.95 * len(ordered)) - 1)]

    def record_snapshot(self, payload_sizes: list[int]) -> None:
        if any(not isinstance(size, int) or size < 0 for size in payload_sizes):
            raise ValueError("snapshot payload sizes must be non-negative integers")
        self.snapshot_size_bytes_max_last_tick = max(payload_sizes, default=0)
        self.snapshot_size_bytes_avg_last_tick = (
            sum(payload_sizes) / len(payload_sizes) if payload_sizes else 0.0
        )
        self.snapshot_bytes_enqueued_total += sum(payload_sizes)
        now = monotonic()
        self.snapshot_messages += len(payload_sizes)
        self.snapshot_send_samples.extend((now, size) for size in payload_sizes)
        while self.snapshot_send_samples and now - self.snapshot_send_samples[0][0] > 5.0:
            self.snapshot_send_samples.popleft()

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
        recent_snapshot_samples = [
            (timestamp, size)
            for timestamp, size in self.snapshot_send_samples
            if now - timestamp <= 1.0
        ]
        average_ms = (
            sum(self.tick_durations_ms) / len(self.tick_durations_ms)
            if self.tick_durations_ms
            else 0.0
        )
        metrics: dict[str, int | float] = {
            "server_tick_duration_ms_avg": round(average_ms, 3),
            "server_tick_duration_ms_last": round(
                self.tick_durations_ms[-1] if self.tick_durations_ms else 0.0, 3
            ),
            "server_tick_duration_ms_p95": round(self._p95(self.tick_durations_ms), 3),
            "server_tick_rate_target": tick_rate,
            "server_tick_rate_observed_1s_min_room": min(room_tick_rates, default=0),
            "room_count": room_count,
            "player_count": player_count,
            "websocket_connections": self.websocket_connections,
            "snapshot_size_bytes_last": self.snapshot_size_bytes_max_last_tick,
            "snapshot_size_bytes_avg_last_tick": round(self.snapshot_size_bytes_avg_last_tick, 3),
            "snapshot_enqueue_rate_1s": len(recent_snapshot_samples),
            "snapshot_bytes_enqueued_per_second": sum(size for _, size in recent_snapshot_samples),
            "snapshot_messages_enqueued_total": self.snapshot_messages,
            "snapshot_bytes_enqueued_total": self.snapshot_bytes_enqueued_total,
            "command_queue_size": sum(self.command_queue_sizes.values()),
        }
        for phase, durations in self.phase_durations_ms.items():
            average = sum(durations) / len(durations) if durations else 0.0
            metrics[f"{phase}_duration_ms_avg"] = round(average, 3)
            metrics[f"{phase}_duration_ms_last"] = round(durations[-1] if durations else 0.0, 3)
            metrics[f"{phase}_duration_ms_p95"] = round(self._p95(durations), 3)
        return metrics
