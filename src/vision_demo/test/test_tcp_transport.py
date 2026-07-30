"""Tests for TCP framing and automatic WebSocket fallback."""

import socket
import struct

import pytest

from vision_demo.transport.base import Transport
from vision_demo.transport.fallback import FallbackTransport
from vision_demo.transport.tcp import TcpTransport


class FakeTransport(Transport):
    def __init__(self, connect_error=None):
        self.connect_error = connect_error
        self.connected = False
        self.closed = False
        self.sent = []
        self.inbound = []

    def connect(self):
        if self.connect_error is not None:
            raise self.connect_error
        self.connected = True

    def send(self, packet):
        if not self.connected:
            raise ConnectionError
        self.sent.append(packet)

    def receive(self):
        if not self.connected:
            raise ConnectionError
        if not self.inbound:
            return None
        return self.inbound.pop(0)

    def close(self):
        self.connected = False
        self.closed = True


def socket_pair_transport():
    client, server = socket.socketpair()
    client.settimeout(0.01)
    server.settimeout(0.1)
    transport = TcpTransport(
        'unused',
        receive_timeout=0.01,
    )
    transport._socket = client
    return transport, server


def frame(packet):
    return struct.pack('<H', len(packet)) + packet


def test_tcp_handles_fragmented_and_coalesced_packets():
    transport, server = socket_pair_transport()
    first = b'first'
    second = b'second'

    try:
        first_frame = frame(first)
        server.sendall(first_frame[:1])
        assert transport.receive() is None

        server.sendall(
            first_frame[1:] + frame(second)
        )
        assert transport.receive() == first
        assert transport.receive() == second
    finally:
        transport.close()
        server.close()


def test_tcp_send_uses_little_endian_length():
    transport, server = socket_pair_transport()

    try:
        transport.send(b'abc')
        assert server.recv(5) == b'\x03\x00abc'
    finally:
        transport.close()
        server.close()


def test_tcp_rejects_invalid_length_and_closes():
    transport, server = socket_pair_transport()

    try:
        server.sendall(b'\x00\x00')

        with pytest.raises(
            ConnectionError,
            match='Invalid TCP packet length',
        ):
            transport.receive()

        assert transport._socket is None
    finally:
        transport.close()
        server.close()


def test_fallback_prefers_primary():
    primary = FakeTransport()
    fallback = FakeTransport()
    transport = FallbackTransport(
        lambda: primary,
        lambda: fallback,
    )

    transport.connect()
    transport.send(b'packet')

    assert primary.sent == [b'packet']
    assert fallback.sent == []
    assert transport.active_name == 'FakeTransport'


def test_fallback_uses_tcp_when_primary_fails():
    primary = FakeTransport(
        ConnectionError('WebSocket unavailable')
    )
    fallback = FakeTransport()
    fallback.inbound.append(b'reply')
    transport = FallbackTransport(
        lambda: primary,
        lambda: fallback,
    )

    transport.connect()
    transport.send(b'command')

    assert primary.closed
    assert fallback.sent == [b'command']
    assert transport.receive() == b'reply'
