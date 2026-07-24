"""Unit tests for the transport-independent V1 protocol codec."""

import pytest

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MAX_PAYLOAD_SIZE,
    MessageFlag,
    MotionMovePayload,
    MotionOpcode,
    NodeId,
    ProtocolError,
    SequenceGenerator,
    ServiceId,
)


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


def test_sequence_generator_wraps():
    """Sequence numbers must wrap from 65535 back to zero."""
    generator = SequenceGenerator(initial=65535)

    assert generator.next_seq() == 65535
    assert generator.next_seq() == 0
    assert generator.next_seq() == 1
