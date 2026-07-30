"""Bounded, thread-safe retry state for reliable V1 commands."""

import threading
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from vision_demo.reliable_protocol import ResultStage


RequestKey = Tuple[int, int]


@dataclass
class PendingRequest:
    """One immutable command retained until its final result."""

    epoch: int
    request_id: int
    dst: int
    service: int
    opcode: int
    packet: bytes
    attempts: int
    deadline: float
    stage: Optional[ResultStage] = None


class RetryManager:
    """Own a fixed number of pending commands and retry deadlines."""

    def __init__(
        self,
        capacity: int = 16,
        retry_timeout_sec: float = 0.25,
        max_retries: int = 3,
    ):
        if capacity <= 0:
            raise ValueError('capacity must be positive')
        if retry_timeout_sec <= 0.0:
            raise ValueError('retry_timeout_sec must be positive')
        if max_retries < 0:
            raise ValueError('max_retries must not be negative')

        self.capacity = capacity
        self.retry_timeout_sec = retry_timeout_sec
        self.max_retries = max_retries
        self._pending: Dict[RequestKey, PendingRequest] = {}
        self._lock = threading.Lock()
        self.retry_count = 0
        self.timeout_count = 0
        self.rejected_full = 0

    @property
    def pending_count(self) -> int:
        """Return current bounded occupancy."""
        with self._lock:
            return len(self._pending)

    def add(self, request: PendingRequest) -> bool:
        """Insert a new request unless the bounded table is full."""
        key = (request.epoch, request.request_id)

        with self._lock:
            if key in self._pending:
                return False
            if len(self._pending) >= self.capacity:
                self.rejected_full += 1
                return False

            self._pending[key] = request
            return True

    def remove(self, epoch: int, request_id: int) -> None:
        """Forget one request, usually after enqueue rollback."""
        with self._lock:
            self._pending.pop((epoch, request_id), None)

    def find(
        self,
        epoch: int,
        request_id: int,
    ) -> Optional[PendingRequest]:
        """Return the live pending object for inspection."""
        with self._lock:
            return self._pending.get((epoch, request_id))

    def observe_result(
        self,
        epoch: int,
        request_id: int,
        stage: ResultStage,
        now: float,
    ) -> Optional[PendingRequest]:
        """Advance one request and remove it after a final result."""
        key = (epoch, request_id)

        with self._lock:
            request = self._pending.get(key)

            if request is None:
                return None

            request.stage = stage
            request.deadline = now + self.retry_timeout_sec

            if stage in (ResultStage.APPLIED, ResultStage.FAILED):
                return self._pending.pop(key)

            return request

    def take_due(self, now: float) -> List[bytes]:
        """Return retry packets and expire requests at their fixed limit."""
        due_packets: List[bytes] = []

        with self._lock:
            for key, request in tuple(self._pending.items()):
                if now < request.deadline:
                    continue

                if request.attempts > self.max_retries:
                    del self._pending[key]
                    self.timeout_count += 1
                    continue

                request.attempts += 1
                request.deadline = now + self.retry_timeout_sec
                due_packets.append(request.packet)
                self.retry_count += 1

        return due_packets
