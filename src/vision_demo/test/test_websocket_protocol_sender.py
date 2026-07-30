"""Tests for V1 packets emitted by the RobotLink control producer."""

from types import SimpleNamespace

from vision_demo.gimbal_pd_websocket_node import (
    GimbalPDWebSocketNode,
)
from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionMovePayload,
    MotionOpcode,
    NodeId,
    SequenceGenerator,
    ServiceId,
)


def test_encode_move_uses_one_realtime_v1_packet():
    """A MOVE command must become one real-time application packet."""
    sender = SimpleNamespace(
        sequence=SequenceGenerator(initial=41)
    )
    payload = MotionMovePayload(
        control_epoch=3,
        valid_ms=300,
        linear_mm_s=0,
        angular_mrad_s=0,
        head_yaw_rate_x10=-200,
        head_pitch_rate_x10=100,
    ).encode()

    seq, packet = GimbalPDWebSocketNode.encode_command(
        sender,
        (MotionOpcode.MOVE, payload),
    )

    assert seq == 41

    message = ApplicationMessage.decode(packet)

    assert message.flags == int(MessageFlag.REALTIME)
    assert message.src == int(NodeId.LINUX)
    assert message.dst == int(NodeId.STM32)
    assert message.service == int(ServiceId.MOTION)
    assert message.opcode == int(MotionOpcode.MOVE)
    assert message.seq == 41
    assert MotionMovePayload.decode(message.payload) == (
        MotionMovePayload(
            control_epoch=3,
            valid_ms=300,
            linear_mm_s=0,
            angular_mrad_s=0,
            head_yaw_rate_x10=-200,
            head_pitch_rate_x10=100,
        )
    )


def test_encode_stop_requests_ack_with_empty_payload():
    """A normal STOP must request acknowledgement and carry no payload."""
    sender = SimpleNamespace(
        sequence=SequenceGenerator(initial=7)
    )
    seq, packet = GimbalPDWebSocketNode.encode_command(
        sender,
        (MotionOpcode.STOP, b''),
    )

    assert seq == 7

    message = ApplicationMessage.decode(packet)

    assert message.flags == int(MessageFlag.ACK_REQUIRED)
    assert message.opcode == int(MotionOpcode.STOP)
    assert message.payload == b''
