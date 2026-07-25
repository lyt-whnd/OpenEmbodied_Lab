"""Tests for keyboard-to-V1 head motion commands."""

from vision_demo.keyboard_motion_node import (
    DIRECTION_DOWN,
    DIRECTION_LEFT,
    DIRECTION_RIGHT,
    DIRECTION_UP,
    KeySequenceDecoder,
    direction_to_head_rates,
    encode_motion_command,
)
from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionOpcode,
    NodeId,
    ServiceId,
)


def test_direction_mapping_matches_existing_servo_convention():
    """Arrow keys must keep the established yaw and pitch signs."""
    assert direction_to_head_rates(DIRECTION_UP, 300) == (
        0,
        -300,
    )
    assert direction_to_head_rates(DIRECTION_DOWN, 300) == (
        0,
        300,
    )
    assert direction_to_head_rates(DIRECTION_LEFT, 300) == (
        300,
        0,
    )
    assert direction_to_head_rates(DIRECTION_RIGHT, 300) == (
        -300,
        0,
    )


def test_arrow_decoder_handles_fragmented_terminal_sequence():
    """An arrow escape sequence may arrive in multiple reads."""
    decoder = KeySequenceDecoder()

    assert decoder.feed('\x1b') == []
    assert decoder.feed('[A') == [DIRECTION_UP]
    assert decoder.feed('bq') == ['b', 'q']


def test_move_command_targets_stm32_through_v1_router():
    """The keyboard packet must be a Linux-to-STM32 V1 message."""
    packet = encode_motion_command(
        MotionOpcode.MOVE,
        0x1234,
        bytes(12),
    )
    message = ApplicationMessage.decode(packet)

    assert message.flags == int(MessageFlag.REALTIME)
    assert message.src == int(NodeId.LINUX)
    assert message.dst == int(NodeId.STM32)
    assert message.service == int(ServiceId.MOTION)
    assert message.opcode == int(MotionOpcode.MOVE)
    assert message.seq == 0x1234


def test_center_command_is_reliable_and_has_no_payload():
    """CENTER must request a response and remain idempotent."""
    packet = encode_motion_command(
        MotionOpcode.CENTER,
        9,
    )
    message = ApplicationMessage.decode(packet)

    assert message.flags == int(MessageFlag.ACK_REQUIRED)
    assert message.opcode == int(MotionOpcode.CENTER)
    assert message.payload == b''
