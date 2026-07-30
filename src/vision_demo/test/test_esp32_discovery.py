"""Unit tests for ESP32-CAM UDP discovery parsing."""

import json

import pytest

from vision_demo.esp32_discovery import (
    DiscoveryError,
    parse_discovery_datagram,
)


def _valid_document():
    return {
        'magic': 'ROBOT_HELLO',
        'proto': 1,
        'name': 'robot-v1',
        'node': 'esp32cam',
        'device_id': 'ESP32CAM_A1B2C3',
        'ip': '10.0.0.99',
        'ws_port': 80,
        'ws_path': '/ws',
        'tcp_port': 9000,
        'stream_port': 81,
        'stream_path': '/stream',
    }


def _encode(document):
    return json.dumps(document).encode('utf-8')


def test_valid_datagram_builds_endpoints_from_udp_source():
    """The UDP source address overrides the untrusted advertised IP."""
    robot = parse_discovery_datagram(
        _encode(_valid_document()),
        ('192.168.103.42', 50999),
    )

    assert robot.ip == '192.168.103.42'
    assert robot.device_id == 'ESP32CAM_A1B2C3'
    assert robot.websocket_url == (
        'ws://192.168.103.42:80/ws'
    )
    assert robot.stream_url == (
        'http://192.168.103.42:81/stream'
    )
    assert robot.tcp_url == (
        'tcp://192.168.103.42:9000'
    )


def test_legacy_discovery_defaults_tcp_port():
    document = _valid_document()
    del document['tcp_port']

    robot = parse_discovery_datagram(
        _encode(document),
        ('192.168.103.42', 50999),
    )

    assert robot.tcp_port == 9000


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('magic', 'OTHER_DEVICE', 'magic'),
        ('proto', 2, 'protocol version'),
        ('ws_port', 0, 'ws_port'),
        ('tcp_port', 0, 'tcp_port'),
        ('stream_port', 70000, 'stream_port'),
        ('ws_path', 'ws', 'ws_path'),
        ('stream_path', '/bad path', 'stream_path'),
    ],
)
def test_invalid_discovery_fields_are_rejected(
    field,
    value,
    message,
):
    """Malformed endpoint metadata must never reach connection code."""
    document = _valid_document()
    document[field] = value

    with pytest.raises(DiscoveryError, match=message):
        parse_discovery_datagram(
            _encode(document),
            ('192.168.1.20', 4210),
        )


def test_non_json_datagram_is_rejected():
    """Unrelated UDP traffic on port 4210 is ignored safely."""
    with pytest.raises(DiscoveryError, match='valid JSON'):
        parse_discovery_datagram(
            b'not-json',
            ('192.168.1.20', 4210),
        )


def test_non_ipv4_source_is_rejected():
    """Discovery V1 accepts only IPv4 LAN source addresses."""
    with pytest.raises(DiscoveryError, match='IPv4'):
        parse_discovery_datagram(
            _encode(_valid_document()),
            ('not-an-ip', 4210),
        )
