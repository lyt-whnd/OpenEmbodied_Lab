"""Linux endpoint for reliable V1 commands with bounded retries."""

import threading
import time
from typing import Callable, Optional, Tuple

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    NodeId,
    ProtocolError,
    SequenceGenerator,
)
from vision_demo.reliable_protocol import (
    ReliableRequest,
    ReliableResult,
    ResultStage,
)
from vision_demo.retry_manager import PendingRequest, RetryManager
from vision_demo.service_registry import QosClass, require_policy


PacketEnqueue = Callable[[bytes], bool]
Clock = Callable[[], float]
ResultCallback = Callable[
    [ApplicationMessage, ReliableResult],
    None,
]

DEFAULT_PENDING_CAPACITY = 8
DEFAULT_RETRY_TIMEOUT_SEC = 0.25
DEFAULT_MAX_RETRIES = 3


class ReliableSender:
    """Create, retain, retry, and resolve critical commands."""

    def __init__(
        self,
        enqueue: PacketEnqueue,
        *,
        epoch: int,
        clock: Clock = time.monotonic,
        capacity: int = DEFAULT_PENDING_CAPACITY,
        retry_timeout_sec: float = DEFAULT_RETRY_TIMEOUT_SEC,
        max_retries: int = DEFAULT_MAX_RETRIES,
        result_callback: Optional[ResultCallback] = None,
    ):
        if not 0 <= epoch <= 0xFFFF:
            raise ValueError('epoch must fit uint16')

        self._enqueue = enqueue
        self._epoch = epoch
        self._clock = clock
        self._sequence = SequenceGenerator()
        self._next_request_id = 0
        self._id_lock = threading.Lock()
        self._result_callback = result_callback
        self.retry_manager = RetryManager(
            capacity=capacity,
            retry_timeout_sec=retry_timeout_sec,
            max_retries=max_retries,
        )
        self.invalid_results = 0
        self.unmatched_results = 0

    @property
    def epoch(self) -> int:
        """Return the sender restart epoch carried by every request."""
        return self._epoch

    def send(
        self,
        *,
        dst: int,
        service: int,
        opcode: int,
        payload: bytes = b'',
    ) -> Optional[Tuple[int, int]]:
        """Enqueue one registered RELIABLE command."""
        policy = require_policy(service, opcode)

        if policy.qos is not QosClass.RELIABLE:
            raise ValueError('message policy is not RELIABLE')
        if int(dst) == int(NodeId.BROADCAST):
            raise ValueError(
                'RELIABLE requests require one concrete destination'
            )

        with self._id_lock:
            request_id = self._next_request_id
            self._next_request_id = (
                self._next_request_id + 1
            ) & 0xFFFFFFFF
            sequence = self._sequence.next_seq()

        request_payload = ReliableRequest(
            epoch=self._epoch,
            request_id=request_id,
            payload=bytes(payload),
        ).encode()
        message = ApplicationMessage(
            flags=int(MessageFlag.ACK_REQUIRED),
            src=int(NodeId.LINUX),
            dst=int(dst),
            service=int(service),
            opcode=int(opcode),
            seq=sequence,
            payload=request_payload,
        )
        packet = message.encode()
        now = self._clock()
        pending = PendingRequest(
            epoch=self._epoch,
            request_id=request_id,
            dst=int(dst),
            service=int(service),
            opcode=int(opcode),
            packet=packet,
            attempts=1,
            deadline=now + self.retry_manager.retry_timeout_sec,
        )

        if not self.retry_manager.add(pending):
            return None

        if not self._enqueue(packet):
            self.retry_manager.remove(self._epoch, request_id)
            return None

        return self._epoch, request_id

    def handle_message(
        self,
        message: ApplicationMessage,
        _packet: bytes,
    ) -> None:
        """Consume matching RESULT messages observed by the router."""
        if not message.flags & int(MessageFlag.RESPONSE):
            return

        try:
            result = ReliableResult.decode(message.payload)
        except (ProtocolError, TypeError, ValueError):
            return

        result_is_error = (
            result.stage is ResultStage.FAILED
            or result.status != 0
        )

        if bool(
            message.flags & int(MessageFlag.ERROR)
        ) != result_is_error:
            self.invalid_results += 1
            return

        pending = self.retry_manager.find(
            result.epoch,
            result.request_id,
        )

        if pending is None:
            self.unmatched_results += 1
            return

        if (
            message.src != pending.dst
            or message.dst != int(NodeId.LINUX)
            or message.service != pending.service
            or message.opcode != pending.opcode
        ):
            self.invalid_results += 1
            return

        resolved = self.retry_manager.observe_result(
            result.epoch,
            result.request_id,
            result.stage,
            self._clock(),
        )

        if resolved is not None and self._result_callback is not None:
            self._result_callback(message, result)

    def poll(self) -> None:
        """Queue every retry whose timeout has expired."""
        for packet in self.retry_manager.take_due(self._clock()):
            self._enqueue(packet)
