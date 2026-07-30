"""Discover ESP32-CAM robot gateways through UDP broadcasts."""

import argparse
import ipaddress
import json
import socket
import time
from dataclasses import asdict, dataclass
from typing import Iterator, Optional, Tuple


DISCOVERY_MAGIC = 'ROBOT_HELLO'
DISCOVERY_PROTOCOL_VERSION = 1
DISCOVERY_PORT = 4210
MAX_DATAGRAM_SIZE = 2048


class DiscoveryError(ValueError):
    """Raised when a UDP discovery datagram is malformed."""


def _required_string(
    document: dict,
    field: str,
    max_length: int,
) -> str:
    value = document.get(field)

    if (
        not isinstance(value, str)
        or not value
        or len(value) > max_length
    ):
        raise DiscoveryError(f'invalid {field}')

    return value


def _required_port(
    document: dict,
    field: str,
    default=None,
) -> int:
    value = document.get(field, default)

    if (
        isinstance(value, bool)
        or not isinstance(value, int)
        or not 1 <= value <= 65535
    ):
        raise DiscoveryError(f'invalid {field}')

    return value


def _required_path(document: dict, field: str) -> str:
    value = _required_string(document, field, 128)

    if not value.startswith('/') or any(
        character.isspace()
        for character in value
    ):
        raise DiscoveryError(f'invalid {field}')

    return value


@dataclass(frozen=True)
class DiscoveredRobot:
    """Validated service endpoints advertised by one ESP32-CAM."""

    device_id: str
    name: str
    node: str
    ip: str
    ws_port: int
    ws_path: str
    tcp_port: int
    stream_port: int
    stream_path: str
    proto: int = DISCOVERY_PROTOCOL_VERSION

    @property
    def websocket_url(self) -> str:
        """Return the V1 binary control endpoint."""
        return f'ws://{self.ip}:{self.ws_port}{self.ws_path}'

    @property
    def stream_url(self) -> str:
        """Return the HTTP MJPEG endpoint."""
        return (
            f'http://{self.ip}:{self.stream_port}'
            f'{self.stream_path}'
        )

    @property
    def tcp_url(self) -> str:
        """Return the raw V1 TCP fallback endpoint."""
        return f'tcp://{self.ip}:{self.tcp_port}'

    def as_output_dict(self) -> dict:
        """Return discovery fields plus ready-to-use endpoint URLs."""
        output = asdict(self)
        output['websocket_url'] = self.websocket_url
        output['tcp_url'] = self.tcp_url
        output['stream_url'] = self.stream_url
        return output


def parse_discovery_datagram(
    datagram: bytes,
    source_address: Tuple[str, int],
) -> DiscoveredRobot:
    """Validate one broadcast and trust its UDP source as the device IP."""
    if not isinstance(datagram, (bytes, bytearray, memoryview)):
        raise DiscoveryError('datagram must be bytes-like')

    if len(datagram) > MAX_DATAGRAM_SIZE:
        raise DiscoveryError('discovery datagram is too large')

    try:
        document = json.loads(bytes(datagram).decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DiscoveryError('discovery datagram is not valid JSON') from error

    if not isinstance(document, dict):
        raise DiscoveryError('discovery JSON must be an object')

    if document.get('magic') != DISCOVERY_MAGIC:
        raise DiscoveryError('invalid discovery magic')

    proto = document.get('proto')

    if (
        isinstance(proto, bool)
        or proto != DISCOVERY_PROTOCOL_VERSION
    ):
        raise DiscoveryError('unsupported discovery protocol version')

    try:
        source_ip = str(ipaddress.IPv4Address(source_address[0]))
    except ipaddress.AddressValueError as error:
        raise DiscoveryError('invalid UDP source IPv4 address') from error

    return DiscoveredRobot(
        device_id=_required_string(
            document,
            'device_id',
            64,
        ),
        name=_required_string(document, 'name', 64),
        node=_required_string(document, 'node', 32),
        ip=source_ip,
        ws_port=_required_port(document, 'ws_port'),
        ws_path=_required_path(document, 'ws_path'),
        tcp_port=_required_port(
            document,
            'tcp_port',
            9000,
        ),
        stream_port=_required_port(
            document,
            'stream_port',
        ),
        stream_path=_required_path(
            document,
            'stream_path',
        ),
        proto=proto,
    )


def listen_for_robots(
    port: int = DISCOVERY_PORT,
    timeout: Optional[float] = None,
    device_id: Optional[str] = None,
    verbose: bool = False,
) -> Iterator[DiscoveredRobot]:
    """Yield valid robot advertisements until the optional timeout."""
    if not 1 <= port <= 65535:
        raise ValueError('port must be in [1, 65535]')

    if timeout is not None and timeout <= 0:
        raise ValueError('timeout must be greater than zero')

    deadline = (
        None
        if timeout is None
        else time.monotonic() + timeout
    )

    with socket.socket(
        socket.AF_INET,
        socket.SOCK_DGRAM,
    ) as discovery_socket:
        discovery_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1,
        )
        discovery_socket.bind(('', port))

        while True:
            if deadline is not None:
                remaining = deadline - time.monotonic()

                if remaining <= 0:
                    return

                discovery_socket.settimeout(remaining)

            try:
                datagram, source_address = (
                    discovery_socket.recvfrom(
                        MAX_DATAGRAM_SIZE + 1
                    )
                )
            except socket.timeout:
                return

            try:
                robot = parse_discovery_datagram(
                    datagram,
                    source_address,
                )
            except DiscoveryError as error:
                if verbose:
                    print(
                        'Ignored invalid discovery packet '
                        f'from {source_address[0]}: {error}'
                    )
                continue

            if (
                device_id is not None
                and robot.device_id != device_id
            ):
                continue

            yield robot


def _build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            'Listen for ESP32-CAM ROBOT_HELLO broadcasts '
            'and print ready-to-use endpoints.'
        )
    )
    parser.add_argument(
        '--port',
        type=int,
        default=DISCOVERY_PORT,
        help='UDP port to listen on (default: 4210)',
    )
    parser.add_argument(
        '--timeout',
        type=float,
        default=None,
        help='overall wait time in seconds (default: wait forever)',
    )
    parser.add_argument(
        '--device-id',
        default=None,
        help='only accept one exact ESP32 device_id',
    )
    parser.add_argument(
        '--once',
        action='store_true',
        help='exit after the first matching robot',
    )
    parser.add_argument(
        '--json',
        action='store_true',
        help='print one machine-readable JSON object per device',
    )
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='report malformed datagrams that are ignored',
    )
    return parser


def main(args=None) -> int:
    """Run the UDP robot discovery command-line listener."""
    options = _build_argument_parser().parse_args(args)
    seen_endpoints = {}
    found = False

    try:
        robots = listen_for_robots(
            port=options.port,
            timeout=options.timeout,
            device_id=options.device_id,
            verbose=options.verbose,
        )

        for robot in robots:
            endpoint_key = (
                robot.ip,
                robot.ws_port,
                robot.ws_path,
                robot.stream_port,
                robot.stream_path,
            )

            if seen_endpoints.get(robot.device_id) == endpoint_key:
                if options.once:
                    return 0
                continue

            seen_endpoints[robot.device_id] = endpoint_key
            found = True

            if options.json:
                print(
                    json.dumps(
                        robot.as_output_dict(),
                        ensure_ascii=False,
                        sort_keys=True,
                    )
                )
            else:
                print(f'device_id: {robot.device_id}')
                print(f'ip:        {robot.ip}')
                print(f'WebSocket: {robot.websocket_url}')
                print(f'MJPEG:     {robot.stream_url}')

            if options.once:
                return 0
    except KeyboardInterrupt:
        return 130

    return 0 if found else 1


if __name__ == '__main__':
    raise SystemExit(main())
