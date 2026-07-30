"""WebSocket implementation of the RobotLink transport contract."""

from typing import Optional

from vision_demo.transport.base import Transport

import websocket


class WebSocketTransport(Transport):
    """Carry one V1 packet in each binary WebSocket frame."""

    def __init__(
        self,
        url: str,
        connect_timeout: float = 2.0,
        receive_timeout: float = 0.05,
    ):
        """Store connection settings without opening a socket."""
        self._url = url
        self._connect_timeout = connect_timeout
        self._receive_timeout = receive_timeout
        self._connection = None

    def connect(self) -> None:
        """Open the single WebSocket owned by this transport."""
        self.close()
        self._connection = websocket.create_connection(
            self._url,
            timeout=self._connect_timeout,
            enable_multithread=False,
        )
        self._connection.settimeout(self._receive_timeout)

    def send(self, packet: bytes) -> None:
        """Send one binary frame."""
        if self._connection is None:
            raise ConnectionError('WebSocket is not connected')

        self._connection.send_binary(packet)

    def receive(self) -> Optional[bytes]:
        """Receive one binary frame while treating timeout as no data."""
        if self._connection is None:
            raise ConnectionError('WebSocket is not connected')

        try:
            packet = self._connection.recv()
        except websocket.WebSocketTimeoutException:
            return None

        if packet is None or packet == b'' or packet == '':
            raise ConnectionError('WebSocket closed')

        if isinstance(packet, str):
            return None

        return bytes(packet)

    def close(self) -> None:
        """Close the current socket without masking the original error."""
        connection = self._connection
        self._connection = None

        if connection is None:
            return

        try:
            connection.close()
        except Exception:
            pass
