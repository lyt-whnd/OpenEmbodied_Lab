"""Tests for the single bounded full-duplex RobotLink."""

import queue
import threading
import time

import pytest

from vision_demo.message_router import MessageRouter
from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionOpcode,
    NodeId,
    ServiceId,
    SystemOpcode,
)
from vision_demo.robot_link import RobotLink, SendPolicy
from vision_demo.transport.base import Transport
from vision_demo.transport.websocket import WebSocketTransport


def _message(
    seq: int,
    *,
    src: NodeId = NodeId.LINUX,
    dst: NodeId = NodeId.STM32,
    service: ServiceId = ServiceId.MOTION,
    opcode: int = MotionOpcode.STOP,
    flags: MessageFlag = MessageFlag.ACK_REQUIRED,
) -> ApplicationMessage:
    return ApplicationMessage(
        flags=int(flags),
        src=int(src),
        dst=int(dst),
        service=int(service),
        opcode=int(opcode),
        seq=seq,
    )


class _FakeTransport(Transport):
    def __init__(self):
        self.connect_calls = 0
        self.close_calls = 0
        self.active_connections = 0
        self.max_active_connections = 0
        self.sent = []
        self.incoming = queue.Queue()
        self._closed = False

    def connect(self) -> None:
        self.connect_calls += 1
        self._closed = False
        self.active_connections += 1
        self.max_active_connections = max(
            self.max_active_connections,
            self.active_connections,
        )

    def send(self, packet: bytes) -> None:
        if self._closed:
            raise ConnectionError('closed')
        self.sent.append(packet)

    def receive(self):
        if self._closed:
            raise ConnectionError('closed')

        try:
            return self.incoming.get(timeout=0.01)
        except queue.Empty:
            return None

    def close(self) -> None:
        if self._closed:
            return

        self._closed = True
        self.close_calls += 1
        self.active_connections -= 1


class _ConnectFailureTransport(Transport):
    def connect(self) -> None:
        raise ConnectionError('offline')

    def send(self, packet: bytes) -> None:
        raise AssertionError('send must not run while offline')

    def receive(self):
        raise AssertionError('receive must not run while offline')

    def close(self) -> None:
        pass


def _wait_for(predicate, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)

    raise AssertionError('condition was not reached before timeout')


def test_fifo_is_bounded_and_latest_value_replaces_old_move():
    """FIFO must reject overflow while best-effort keeps only newest."""
    link = RobotLink(
        lambda: _FakeTransport(),
        queue_size=1,
    )
    first = _message(1).encode()
    second = _message(2).encode()

    assert link.send_packet(first, SendPolicy.FIFO)
    assert not link.send_packet(second, SendPolicy.FIFO)
    assert link.queue_drops == 1

    assert link._take_next_packet() == first
    assert link.send_packet(first, SendPolicy.BEST_EFFORT)
    assert link.send_packet(second, SendPolicy.BEST_EFFORT)
    assert link.best_effort_replacements == 1
    assert link._take_next_packet() == second


def test_reconnect_boundary_discards_stale_best_effort():
    """A MOVE waiting across reconnect must never be replayed."""
    connected_transport = _FakeTransport()
    transports = iter([
        _ConnectFailureTransport(),
        connected_transport,
    ])
    link = RobotLink(
        lambda: next(transports),
        reconnect_delay_sec=0.1,
    )
    packet = _message(
        3,
        flags=MessageFlag.REALTIME,
    ).encode()

    link.start()
    _wait_for(lambda: link.disconnect_count == 1)
    assert link.send_packet(packet, SendPolicy.BEST_EFFORT)
    _wait_for(lambda: link.connected)
    time.sleep(0.05)
    link.stop()

    assert connected_transport.sent == []


def test_one_worker_owns_one_full_duplex_connection():
    """One link worker must send and receive through one transport."""
    transport = _FakeTransport()
    created = []
    received = []
    received_event = threading.Event()
    link = RobotLink(
        lambda: created.append(transport) or transport,
        reconnect_delay_sec=0.05,
    )
    link.register_observer(
        lambda message, packet: (
            received.append((message, packet)),
            received_event.set(),
        )
    )

    link.start()
    _wait_for(lambda: link.connected)

    outbound = _message(4).encode()
    inbound = _message(
        5,
        src=NodeId.STM32,
        dst=NodeId.LINUX,
        service=ServiceId.SYSTEM,
        opcode=SystemOpcode.PONG,
        flags=MessageFlag.RESPONSE,
    ).encode()

    assert link.send_packet(outbound, SendPolicy.FIFO)
    transport.incoming.put(inbound)
    _wait_for(lambda: transport.sent == [outbound])
    assert received_event.wait(timeout=1.0)

    link.stop()

    assert len(created) == 1
    assert transport.connect_calls == 1
    assert transport.max_active_connections == 1
    assert received[0][0].seq == 5
    assert received[0][1] == inbound


def test_router_observes_pong_and_stm32_response_after_one_decode():
    """Router handlers and observers must share one decoded object."""
    router = MessageRouter()
    observed = []
    handled = []
    router.register_observer(
        lambda message, packet: observed.append((message, packet))
    )
    router.register_handler(
        int(ServiceId.SYSTEM),
        int(SystemOpcode.PONG),
        lambda message, packet: handled.append((message, packet)),
    )
    packet = _message(
        6,
        src=NodeId.STM32,
        dst=NodeId.LINUX,
        service=ServiceId.SYSTEM,
        opcode=SystemOpcode.PONG,
        flags=MessageFlag.RESPONSE,
    ).encode()

    assert router.route_packet(packet)
    assert len(observed) == 1
    assert len(handled) == 1
    assert observed[0][0] is handled[0][0]
    assert observed[0][1] == packet

    assert not router.route_packet(b'\x01')
    assert router.decode_errors == 1


def test_websocket_empty_receive_is_a_disconnect():
    """An empty recv result must trigger reconnect instead of a busy loop."""
    class _ClosedConnection:
        def recv(self):
            return ''

    transport = WebSocketTransport('ws://unused')
    transport._connection = _ClosedConnection()

    with pytest.raises(ConnectionError):
        transport.receive()
