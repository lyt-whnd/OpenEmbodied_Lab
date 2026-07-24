"""Tests for V1 binary messages emitted by the WebSocket controller."""

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


class _FakeConnection:
    def __init__(self):
        self.sent_binary = []

    def send_binary(self, packet):
        self.sent_binary.append(packet)


def test_send_move_uses_one_binary_v1_frame():
    """A MOVE command must become one real-time binary frame."""
    sender = SimpleNamespace(
        sequence=SequenceGenerator(initial=41)
    )
    connection = _FakeConnection()
    payload = MotionMovePayload(
        control_epoch=3,
        valid_ms=300,
        linear_mm_s=0,
        angular_mrad_s=0,
        head_yaw_rate_x10=-200,
        head_pitch_rate_x10=100,
    ).encode()

    seq = GimbalPDWebSocketNode.send_command(
        sender,
        connection,
        (MotionOpcode.MOVE, payload),
    )

    assert seq == 41
    assert len(connection.sent_binary) == 1

    message = ApplicationMessage.decode(
        connection.sent_binary[0]
    )

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


def test_send_stop_requests_ack_with_empty_payload():
    """A normal STOP must request acknowledgement and carry no payload."""
    sender = SimpleNamespace(
        sequence=SequenceGenerator(initial=7)
    )
    connection = _FakeConnection()

    seq = GimbalPDWebSocketNode.send_command(
        sender,
        connection,
        (MotionOpcode.STOP, b''),
    )

    assert seq == 7

    message = ApplicationMessage.decode(
        connection.sent_binary[0]
    )

    assert message.flags == int(MessageFlag.ACK_REQUIRED)
    assert message.opcode == int(MotionOpcode.STOP)
    assert message.payload == b''
