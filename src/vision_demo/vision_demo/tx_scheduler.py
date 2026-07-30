"""Bounded three-class transmit scheduler shared by Linux transports."""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

from vision_demo.service_registry import MessagePolicy, Priority


class QueueKind(Enum):
    """Storage behavior selected after resolving a message policy."""

    RELIABLE = 'reliable'
    BEST_EFFORT_LATEST = 'best_effort_latest'
    BEST_EFFORT_SAMPLE = 'best_effort_sample'
    BULK = 'bulk'


@dataclass(frozen=True)
class ScheduledPacket:
    packet: bytes
    priority: Priority
    expires_at: Optional[float]


@dataclass
class TxSchedulerStats:
    enqueued: int = 0
    dequeued: int = 0
    dropped_full: int = 0
    replaced_latest: int = 0
    dropped_sample: int = 0
    expired: int = 0


class TxScheduler:
    """Keep each QoS class in separate, fixed-capacity storage."""

    def __init__(
        self,
        *,
        reliable_capacity: int = 8,
        latest_capacity: int = 8,
        sample_capacity: int = 16,
        bulk_capacity: int = 2,
        clock=time.monotonic,
    ):
        capacities = (
            reliable_capacity,
            latest_capacity,
            sample_capacity,
            bulk_capacity,
        )
        if any(capacity <= 0 for capacity in capacities):
            raise ValueError('scheduler capacities must be positive')

        self.reliable_capacity = reliable_capacity
        self.latest_capacity = latest_capacity
        self.sample_capacity = sample_capacity
        self.bulk_capacity = bulk_capacity
        self._clock = clock
        self._reliable: List[ScheduledPacket] = []
        self._latest: Dict[Tuple[int, ...], ScheduledPacket] = {}
        self._samples: List[ScheduledPacket] = []
        self._bulk: List[ScheduledPacket] = []
        self._lock = threading.Lock()
        self.stats = TxSchedulerStats()

    @staticmethod
    def _deadline(
        policy: MessagePolicy,
        now: float,
    ) -> Optional[float]:
        if policy.deadline_ms <= 0:
            return None
        return now + (policy.deadline_ms / 1000.0)

    def enqueue(
        self,
        packet: bytes,
        *,
        kind: QueueKind,
        policy: MessagePolicy,
        key: Tuple[int, ...],
    ) -> bool:
        """Copy one packet into the selected bounded storage."""
        item = ScheduledPacket(
            packet=bytes(packet),
            priority=policy.priority,
            expires_at=self._deadline(policy, self._clock()),
        )

        with self._lock:
            if kind is QueueKind.RELIABLE:
                if len(self._reliable) >= self.reliable_capacity:
                    self.stats.dropped_full += 1
                    return False
                self._reliable.append(item)
            elif kind is QueueKind.BEST_EFFORT_LATEST:
                if key in self._latest:
                    self.stats.replaced_latest += 1
                elif len(self._latest) >= self.latest_capacity:
                    oldest_key = next(iter(self._latest))
                    del self._latest[oldest_key]
                    self.stats.replaced_latest += 1
                self._latest[key] = item
            elif kind is QueueKind.BEST_EFFORT_SAMPLE:
                if len(self._samples) >= self.sample_capacity:
                    self._samples.pop(0)
                    self.stats.dropped_sample += 1
                self._samples.append(item)
            elif kind is QueueKind.BULK:
                if len(self._bulk) >= self.bulk_capacity:
                    self.stats.dropped_full += 1
                    return False
                self._bulk.append(item)
            else:
                raise ValueError(f'unsupported queue kind: {kind}')

            self.stats.enqueued += 1
            return True

    def _take_live(
        self,
        items: List[ScheduledPacket],
        now: float,
        *,
        highest_priority: bool = False,
    ) -> Optional[ScheduledPacket]:
        while items:
            index = 0
            if highest_priority:
                index = max(
                    range(len(items)),
                    key=lambda candidate: items[candidate].priority,
                )
            item = items.pop(index)
            if (
                item.expires_at is not None
                and now >= item.expires_at
            ):
                self.stats.expired += 1
                continue
            return item
        return None

    def take_next(self) -> Optional[bytes]:
        """Return one live packet in strict class-priority order."""
        now = self._clock()

        with self._lock:
            item = self._take_live(
                self._reliable,
                now,
                highest_priority=True,
            )
            if item is None:
                latest_items = list(self._latest.items())
                self._latest.clear()
                latest_items.sort(
                    key=lambda entry: entry[1].priority,
                    reverse=True,
                )
                while latest_items:
                    key, candidate = latest_items.pop(0)
                    if (
                        candidate.expires_at is not None
                        and now >= candidate.expires_at
                    ):
                        self.stats.expired += 1
                        continue
                    item = candidate
                    for pending_key, pending in latest_items:
                        self._latest[pending_key] = pending
                    break
            if item is None:
                item = self._take_live(self._samples, now)
            if item is None:
                item = self._take_live(self._bulk, now)
            if item is None:
                return None

            self.stats.dequeued += 1
            return item.packet

    def discard_best_effort(self) -> None:
        """Remove stale realtime state across connection boundaries."""
        with self._lock:
            self._latest.clear()
            self._samples.clear()

    @property
    def pending_count(self) -> int:
        with self._lock:
            return (
                len(self._reliable)
                + len(self._latest)
                + len(self._samples)
                + len(self._bulk)
            )
