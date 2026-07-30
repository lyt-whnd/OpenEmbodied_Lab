"""Unit tests for the transport-independent V1 protocol codec."""

import json
from pathlib import Path

import pytest

from vision_demo.protocol_v1 import (
    APPLICATION_HEADER_SIZE,
    ApplicationMessage,
    KNOWN_FLAGS_MASK,
    MAX_PAYLOAD_SIZE,
    MessageFlag,
    MotionMovePayload,
    MotionOpcode,
    NodeId,
    PROTOCOL_VERSION,
    ProtocolError,
    SequenceGenerator,
    ServiceId,
    SystemOpcode,
)


_REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
_VECTOR_FILE = (
    _REPOSITORY_ROOT
    / 'protocol_test_vectors'
    / 'v1_vectors.json'
)
_VECTORS = json.loads(_VECTOR_FILE.read_text(encoding='utf-8'))
_ERROR_PATTERNS = {
    'HEADER_TOO_SHORT': 'shorter',
    'UNSUPPORTED_VERSION': 'unsupported',
    'UNKNOWN_FLAGS': 'unknown message flags',
    'PAYLOAD_TOO_LARGE': 'exceeds',
    'LENGTH_MISMATCH': 'payload_len',
}


def test_canonical_registry_constants():
    """Python registry values must match the canonical vector source."""
    assert PROTOCOL_VERSION == _VECTORS['protocolVersion']
    assert APPLICATION_HEADER_SIZE == _VECTORS['headerSize']
    assert MAX_PAYLOAD_SIZE == _VECTORS['maxPayloadSize']
    assert KNOWN_FLAGS_MASK == _VECTORS['knownFlagsMask']
    assert {
        name: int(getattr(MessageFlag, name))
        for name in _VECTORS['flags']
    } == _VECTORS['flags']
    assert {
        name: int(getattr(NodeId, name))
        for name in _VECTORS['nodes']
    } == _VECTORS['nodes']
    assert {
        name: int(getattr(ServiceId, name))
        for name in _VECTORS['services']
    } == _VECTORS['services']
    assert {
        name: int(getattr(SystemOpcode, name))
        for name in _VECTORS['systemOpcodes']
    } == _VECTORS['systemOpcodes']
    assert {
        name: int(getattr(MotionOpcode, name))
        for name in _VECTORS['motionOpcodes']
    } == _VECTORS['motionOpcodes']


@pytest.mark.parametrize(
    'vector',
    _VECTORS['validMessages'],
    ids=lambda vector: vector['name'],
)
def test_canonical_valid_message_vectors(vector):
    """Every canonical V1 vector must decode and encode identically."""
    packet = bytes.fromhex(vector['messageHex'])
    message = ApplicationMessage.decode(packet)

    assert message.version == vector['version']
    assert message.flags == vector['flags']
    assert message.src == vector['src']
    assert message.dst == vector['dst']
    assert message.service == vector['service']
    assert message.opcode == vector['opcode']
    assert message.seq == vector['seq']
    assert message.payload == bytes.fromhex(vector['payloadHex'])
    assert message.encode() == packet


@pytest.mark.parametrize(
    'vector',
    _VECTORS['invalidMessages'],
    ids=lambda vector: vector['name'],
)
def test_canonical_invalid_message_vectors(vector):
    """Every canonical malformed V1 vector must be rejected."""
    packet = bytes.fromhex(vector['messageHex'])

    with pytest.raises(
        ProtocolError,
        match=_ERROR_PATTERNS[vector['expectedError']],
    ):
        ApplicationMessage.decode(packet)


def test_known_motion_message_vector():
    """The documented little-endian motion vector must remain stable."""
    payload = MotionMovePayload(
        control_epoch=2,
        valid_ms=300,
        linear_mm_s=150,
        angular_mrad_s=-300,
        head_yaw_rate_x10=200,
        head_pitch_rate_x10=0,
    ).encode()

    message = ApplicationMessage(
        flags=int(MessageFlag.REALTIME),
        src=int(NodeId.LINUX),
        dst=int(NodeId.STM32),
        service=int(ServiceId.MOTION),
        opcode=int(MotionOpcode.MOVE),
        seq=0x1234,
        payload=payload,
    )

    assert message.encode().hex() == (
        '01080103100134120c00'
        '02002c019600d4fec8000000'
    )


def test_application_message_round_trip():
    """Encoding and decoding must preserve every application field."""
    original = ApplicationMessage(
        flags=int(
            MessageFlag.ACK_REQUIRED
            | MessageFlag.REALTIME
        ),
        src=int(NodeId.LINUX),
        dst=int(NodeId.ESP32),
        service=int(ServiceId.CONFIG),
        opcode=0x22,
        seq=65535,
        payload=b'\x00\x7f\xff',
    )

    assert ApplicationMessage.decode(original.encode()) == original


def test_motion_payload_round_trip():
    """The fixed motion payload must round-trip signed values."""
    original = MotionMovePayload(
        control_epoch=9,
        valid_ms=250,
        linear_mm_s=-120,
        angular_mrad_s=450,
        head_yaw_rate_x10=-75,
        head_pitch_rate_x10=125,
    )

    assert MotionMovePayload.decode(original.encode()) == original


def test_system_pong_vector_matches_esp32():
    """The ESP32 PONG response vector must be stable across languages."""
    message = ApplicationMessage(
        flags=int(MessageFlag.RESPONSE),
        src=int(NodeId.ESP32),
        dst=int(NodeId.LINUX),
        service=int(ServiceId.SYSTEM),
        opcode=int(SystemOpcode.PONG),
        seq=0x1234,
    )

    assert message.encode().hex() == (
        '01020201010234120000'
    )


def test_decode_rejects_length_mismatch():
    """A packet whose payload_len is wrong must be rejected."""
    packet = bytes.fromhex(
        '01000103100201000400'
        '0102'
    )

    with pytest.raises(
        ProtocolError,
        match='payload_len',
    ):
        ApplicationMessage.decode(packet)


def test_encode_rejects_oversized_payload():
    """Payloads larger than the V1 limit must be rejected."""
    message = ApplicationMessage(
        flags=0,
        src=int(NodeId.LINUX),
        dst=int(NodeId.ESP32),
        service=int(ServiceId.SYSTEM),
        opcode=1,
        seq=1,
        payload=bytes(MAX_PAYLOAD_SIZE + 1),
    )

    with pytest.raises(
        ProtocolError,
        match='exceeds',
    ):
        message.encode()


def test_encode_rejects_unknown_flags():
    """Python must reject the same unknown flag bits as C and C++."""
    message = ApplicationMessage(
        flags=KNOWN_FLAGS_MASK | 0x10,
        src=int(NodeId.LINUX),
        dst=int(NodeId.ESP32),
        service=int(ServiceId.SYSTEM),
        opcode=1,
        seq=1,
    )

    with pytest.raises(
        ProtocolError,
        match='unknown message flags',
    ):
        message.encode()


def test_sequence_generator_wraps():
    """Sequence numbers must wrap from 65535 back to zero."""
    generator = SequenceGenerator(initial=65535)

    assert generator.next_seq() == 65535
    assert generator.next_seq() == 0
    assert generator.next_seq() == 1
