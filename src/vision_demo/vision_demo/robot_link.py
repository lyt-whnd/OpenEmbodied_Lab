"""Single full-duplex robot connection with bounded outbound storage."""

import enum
import secrets
import threading
from typing import Callable, Optional

from vision_demo.message_router import MessageHandler, MessageRouter
from vision_demo.protocol_v1 import ApplicationMessage
from vision_demo.reliable_sender import (
    DEFAULT_PENDING_CAPACITY,
    ReliableSender,
)
from vision_demo.service_registry import QosClass, require_policy
from vision_demo.transport.base import Transport
from vision_demo.tx_scheduler import QueueKind, TxScheduler


TransportFactory = Callable[[], Transport]


class SendPolicy(enum.Enum):
    """Compatibility names for the Stage-2 bounded scheduler."""

    BEST_EFFORT_LATEST = 'best_effort_latest'
    BEST_EFFORT_SAMPLE = 'best_effort_sample'
    RELIABLE = 'reliable'
    BULK = 'bulk'
    BEST_EFFORT = 'best_effort_latest'
    FIFO = 'reliable'


class RobotLink:
    """Own one transport connection and its send/receive worker."""

    def __init__(
        self,
        transport_factory: TransportFactory,
        reconnect_delay_sec: float = 1.0,
        queue_size: int = 8,
        router: Optional[MessageRouter] = None,
        reliable_epoch: Optional[int] = None,
    ):
        """Create bounded state without starting a connection."""
        if reconnect_delay_sec <= 0.0:
            raise ValueError('reconnect_delay_sec must be positive')

        if queue_size <= 0:
            raise ValueError('queue_size must be positive')

        self._transport_factory = transport_factory
        self._reconnect_delay_sec = reconnect_delay_sec
        self._scheduler = TxScheduler(
            reliable_capacity=queue_size,
            latest_capacity=queue_size,
            sample_capacity=max(queue_size, 8),
            bulk_capacity=2,
        )
        self._router = router or MessageRouter()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.reliable_sender = ReliableSender(
            enqueue=lambda packet: self.send_packet(
                packet,
                SendPolicy.FIFO,
            ),
            epoch=(
                secrets.randbits(16)
                if reliable_epoch is None
                else reliable_epoch
            ),
            capacity=DEFAULT_PENDING_CAPACITY,
        )
        self._router.register_observer(
            self.reliable_sender.handle_message
        )

        self.connection_count = 0
        self.disconnect_count = 0
        self.sent_count = 0
        self.received_count = 0

    @property
    def connected(self) -> bool:
        """Return whether the worker currently owns a live transport."""
        return self._connected_event.is_set()

    @property
    def queue_capacity(self) -> int:
        """Return the fixed reliable queue capacity."""
        return self._scheduler.reliable_capacity

    def register_handler(
        self,
        service: int,
        opcode: int,
        handler: MessageHandler,
    ) -> None:
        """Register one received-message handler."""
        self._router.register_handler(service, opcode, handler)

    def register_observer(self, observer: MessageHandler) -> None:
        """Register an observer for every valid received packet."""
        self._router.register_observer(observer)

    def start(self) -> None:
        """Start the sole connection worker."""
        if self._thread is not None and self._thread.is_alive():
            return

        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name='robot_link',
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        """Stop the worker without accessing its transport cross-thread."""
        self._stop_event.set()
        thread = self._thread

        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)

        self._connected_event.clear()

        if thread is None or not thread.is_alive():
            self._thread = None

    def send_message(
        self,
        message: ApplicationMessage,
        policy: SendPolicy,
    ) -> bool:
        """Encode and enqueue one application message."""
        return self.send_packet(message.encode(), policy)

    def send_message_auto(
        self,
        message: ApplicationMessage,
    ) -> bool:
        """Select bounded storage from the common message policy."""
        policy = require_policy(message.service, message.opcode)

        if policy.qos is QosClass.BEST_EFFORT:
            return self.send_message(
                message,
                SendPolicy.BEST_EFFORT_LATEST,
            )
        if policy.qos is QosClass.BULK:
            return self.send_message(message, SendPolicy.BULK)
        return self.send_message(message, SendPolicy.RELIABLE)

    def send_reliable(
        self,
        *,
        dst: int,
        service: int,
        opcode: int,
        payload: bytes = b'',
    ):
        """Create and retain one reliable request until final RESULT."""
        return self.reliable_sender.send(
            dst=dst,
            service=service,
            opcode=opcode,
            payload=payload,
        )

    def send_packet(self, packet: bytes, policy: SendPolicy) -> bool:
        """Queue one immutable packet according to a bounded policy."""
        message = ApplicationMessage.decode(packet)
        message_policy = require_policy(
            message.service,
            message.opcode,
        )
        kind = {
            SendPolicy.RELIABLE: QueueKind.RELIABLE,
            SendPolicy.BEST_EFFORT_LATEST:
                QueueKind.BEST_EFFORT_LATEST,
            SendPolicy.BEST_EFFORT_SAMPLE:
                QueueKind.BEST_EFFORT_SAMPLE,
            SendPolicy.BULK: QueueKind.BULK,
        }[policy]
        return self._scheduler.enqueue(
            packet,
            kind=kind,
            policy=message_policy,
            key=(
                message.dst,
                message.service,
                message.opcode,
            ),
        )

    def discard_stale_best_effort(self) -> None:
        """Discard a MOVE/latest-value packet across reconnect boundaries."""
        self._scheduler.discard_best_effort()

    def _take_next_packet(self) -> Optional[bytes]:
        return self._scheduler.take_next()

    @property
    def queue_drops(self) -> int:
        return self._scheduler.stats.dropped_full

    @property
    def best_effort_replacements(self) -> int:
        return self._scheduler.stats.replaced_latest

    @property
    def expired_count(self) -> int:
        return self._scheduler.stats.expired

    def _run(self) -> None:
        while not self._stop_event.is_set():
            transport = self._transport_factory()

            try:
                transport.connect()
                self.connection_count += 1

                # Anything produced while disconnected may already be stale.
                self.discard_stale_best_effort()
                self._connected_event.set()

                self._run_connected(transport)
            except Exception:
                self.disconnect_count += 1
            finally:
                self._connected_event.clear()
                self.discard_stale_best_effort()
                transport.close()

            self._stop_event.wait(self._reconnect_delay_sec)

    def _run_connected(self, transport: Transport) -> None:
        while not self._stop_event.is_set():
            self.reliable_sender.poll()
            packet = self._take_next_packet()

            if packet is not None:
                transport.send(packet)
                self.sent_count += 1

            received = transport.receive()

            if received is None:
                continue

            if self._router.route_packet(received):
                self.received_count += 1
