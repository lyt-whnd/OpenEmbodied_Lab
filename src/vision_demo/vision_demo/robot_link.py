"""Single full-duplex robot connection with bounded outbound storage."""

import enum
import queue
import threading
from typing import Callable, Optional

from vision_demo.message_router import MessageHandler, MessageRouter
from vision_demo.protocol_v1 import ApplicationMessage
from vision_demo.transport.base import Transport


TransportFactory = Callable[[], Transport]


class SendPolicy(enum.Enum):
    """Stage-1 outbound queue behavior."""

    BEST_EFFORT = 'best_effort'
    FIFO = 'fifo'


class RobotLink:
    """Own one transport connection and its send/receive worker."""

    def __init__(
        self,
        transport_factory: TransportFactory,
        reconnect_delay_sec: float = 1.0,
        queue_size: int = 8,
        router: Optional[MessageRouter] = None,
    ):
        """Create bounded state without starting a connection."""
        if reconnect_delay_sec <= 0.0:
            raise ValueError('reconnect_delay_sec must be positive')

        if queue_size <= 0:
            raise ValueError('queue_size must be positive')

        self._transport_factory = transport_factory
        self._reconnect_delay_sec = reconnect_delay_sec
        self._fifo = queue.Queue(maxsize=queue_size)
        self._latest_lock = threading.Lock()
        self._latest_best_effort: Optional[bytes] = None
        self._router = router or MessageRouter()
        self._stop_event = threading.Event()
        self._connected_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self.queue_drops = 0
        self.best_effort_replacements = 0
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
        """Return the fixed FIFO capacity."""
        return self._fifo.maxsize

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

    def send_packet(self, packet: bytes, policy: SendPolicy) -> bool:
        """Queue one immutable packet according to a bounded policy."""
        immutable_packet = bytes(packet)

        if policy is SendPolicy.BEST_EFFORT:
            with self._latest_lock:
                if self._latest_best_effort is not None:
                    self.best_effort_replacements += 1

                self._latest_best_effort = immutable_packet

            return True

        try:
            self._fifo.put_nowait(immutable_packet)
            return True
        except queue.Full:
            self.queue_drops += 1
            return False

    def discard_stale_best_effort(self) -> None:
        """Discard a MOVE/latest-value packet across reconnect boundaries."""
        with self._latest_lock:
            self._latest_best_effort = None

    def _take_next_packet(self) -> Optional[bytes]:
        try:
            return self._fifo.get_nowait()
        except queue.Empty:
            pass

        with self._latest_lock:
            packet = self._latest_best_effort
            self._latest_best_effort = None
            return packet

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
            packet = self._take_next_packet()

            if packet is not None:
                transport.send(packet)
                self.sent_count += 1

            received = transport.receive()

            if received is None:
                continue

            if self._router.route_packet(received):
                self.received_count += 1
