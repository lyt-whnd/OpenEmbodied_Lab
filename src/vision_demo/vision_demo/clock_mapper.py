"""Map a remote 32-bit monotonic millisecond clock onto local ROS time."""

from dataclasses import dataclass
from typing import Optional


UINT32_MODULUS = 1 << 32
UINT32_HALF_RANGE = 1 << 31


@dataclass(frozen=True)
class ClockObservation:
    remote_ms: int
    extended_remote_ms: int
    local_receive_time: float
    mapped_time: float


class ClockMapper:
    """Offset-only mapper with explicit wrap and reconnect handling."""

    def __init__(self, smoothing: float = 0.1):
        if not 0.0 < smoothing <= 1.0:
            raise ValueError('smoothing must be in (0, 1]')
        self.smoothing = smoothing
        self.reset_count = 0
        self._last_remote: Optional[int] = None
        self._wrap_base = 0
        self._offset: Optional[float] = None

    def reset(self) -> None:
        """Forget the device epoch after reconnect or detected restart."""
        self._last_remote = None
        self._wrap_base = 0
        self._offset = None
        self.reset_count += 1

    def _extend(self, remote_ms: int) -> int:
        if not 0 <= remote_ms <= 0xFFFFFFFF:
            raise ValueError('remote timestamp must fit uint32')

        if self._last_remote is not None and remote_ms < self._last_remote:
            backwards = self._last_remote - remote_ms
            if backwards > UINT32_HALF_RANGE:
                self._wrap_base += UINT32_MODULUS
            else:
                # A small backward jump is reordering or a device restart.
                self.reset()

        self._last_remote = remote_ms
        return self._wrap_base + remote_ms

    def observe(
        self,
        remote_ms: int,
        local_receive_time: float,
    ) -> ClockObservation:
        """Record both clocks and update a bounded-complexity offset."""
        extended = self._extend(remote_ms)
        remote_seconds = extended / 1000.0
        measured_offset = local_receive_time - remote_seconds

        if self._offset is None:
            self._offset = measured_offset
        elif measured_offset < self._offset:
            # Queueing delay only increases the measured offset. Follow the
            # lower envelope so delayed batches do not rewrite sample time.
            self._offset += self.smoothing * (
                measured_offset - self._offset
            )

        return ClockObservation(
            remote_ms=remote_ms,
            extended_remote_ms=extended,
            local_receive_time=local_receive_time,
            mapped_time=remote_seconds + self._offset,
        )

    def map_remote(self, remote_ms: int) -> float:
        """Map a timestamp after at least one observation."""
        if self._offset is None:
            raise RuntimeError('clock mapper has no observation')
        extended = self._extend(remote_ms)
        return (extended / 1000.0) + self._offset
