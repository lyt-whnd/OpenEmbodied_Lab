"""Length-prefixed TCP implementation of the RobotLink transport."""

import socket
import struct
from typing import Optional

from vision_demo.transport.base import Transport


LENGTH_PREFIX_SIZE = 2
DEFAULT_MAX_PACKET_SIZE = 266


class TcpTransport(Transport):
    """Carry complete V1 packets over one reliable TCP byte stream."""

    def __init__(
        self,
        host: str,
        port: int = 9000,
        connect_timeout: float = 2.0,
        receive_timeout: float = 0.05,
        max_packet_size: int = DEFAULT_MAX_PACKET_SIZE,
    ):
        self._host = host
        self._port = port
        self._connect_timeout = connect_timeout
        self._receive_timeout = receive_timeout
        self._max_packet_size = max_packet_size
        self._socket: Optional[socket.socket] = None
        self._receive_buffer = bytearray()

    def connect(self) -> None:
        """Open the TCP connection and configure bounded polling."""
        self.close()
        connection = socket.create_connection(
            (self._host, self._port),
            timeout=self._connect_timeout,
        )
        connection.setsockopt(
            socket.IPPROTO_TCP,
            socket.TCP_NODELAY,
            1,
        )
        connection.settimeout(self._receive_timeout)
        self._socket = connection
        self._receive_buffer.clear()

    def send(self, packet: bytes) -> None:
        """Send one length-prefixed application packet."""
        connection = self._require_connection()
        packet_length = len(packet)

        if (
            packet_length == 0
            or packet_length > self._max_packet_size
        ):
            raise ValueError('TCP packet length is outside bounds')

        connection.sendall(
            struct.pack('<H', packet_length) + packet
        )

    def receive(self) -> Optional[bytes]:
        """Return one complete packet, preserving partial TCP input."""
        packet = self._take_buffered_packet()

        if packet is not None:
            return packet

        connection = self._require_connection()

        try:
            chunk = connection.recv(4096)
        except socket.timeout:
            return None

        if chunk == b'':
            raise ConnectionError('TCP connection closed')

        self._receive_buffer.extend(chunk)
        return self._take_buffered_packet()

    def close(self) -> None:
        """Close the socket and discard any partial packet."""
        connection = self._socket
        self._socket = None
        self._receive_buffer.clear()

        if connection is None:
            return

        try:
            connection.close()
        except OSError:
            pass

    def _require_connection(self) -> socket.socket:
        connection = self._socket

        if connection is None:
            raise ConnectionError('TCP transport is not connected')

        return connection

    def _take_buffered_packet(self) -> Optional[bytes]:
        if len(self._receive_buffer) < LENGTH_PREFIX_SIZE:
            return None

        packet_length = struct.unpack_from(
            '<H',
            self._receive_buffer,
        )[0]

        if (
            packet_length == 0
            or packet_length > self._max_packet_size
        ):
            self.close()
            raise ConnectionError('Invalid TCP packet length')

        framed_length = LENGTH_PREFIX_SIZE + packet_length

        if len(self._receive_buffer) < framed_length:
            return None

        packet = bytes(
            self._receive_buffer[
                LENGTH_PREFIX_SIZE:framed_length
            ]
        )
        del self._receive_buffer[:framed_length]
        return packet
