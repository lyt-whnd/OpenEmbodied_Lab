"""Host integration and fault-injection tests for the current V1 path."""

from three_node_sim import (
    MAX_COBS_SIZE,
    Stm32Node,
    ThreeNodeRig,
    encode_uart_frame,
)

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    MessageFlag,
    MotionOpcode,
    NodeId,
    ServiceId,
    SystemOpcode,
)


def motion_message(seq: int) -> ApplicationMessage:
    """Return one minimal Linux-to-STM32 motion message."""
    return ApplicationMessage(
        flags=MessageFlag.REALTIME,
        src=NodeId.LINUX,
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
        seq=seq,
    )


def test_fragmented_linux_gateway_stm32_path() -> None:
    """UART fragmentation must not change one application packet."""
    rig = ThreeNodeRig()
    rig.linux_send(motion_message(10))
    rig.flush_linux_to_stm32((1, 2, 3, 1))
    assert [message.seq for message in rig.stm32.received] == [10]


def test_coalesced_uart_frames_remain_separate() -> None:
    """Adjacent UART frames must remain two application messages."""
    rig = ThreeNodeRig()
    rig.linux_send(motion_message(11))
    rig.linux_send(motion_message(12))
    rig.flush_linux_to_stm32()
    assert [message.seq for message in rig.stm32.received] == [11, 12]


def test_crc_corruption_is_rejected() -> None:
    """A corrupted UART frame must not reach the STM32 application."""
    node = Stm32Node(lambda _: None)
    wire = bytearray(encode_uart_frame(motion_message(13).encode()))
    wire[2] ^= 0x01
    node.input_wire(bytes(wire))
    assert node.received == []
    assert node.uart.rejected_frames == 1


def test_oversized_frame_recovers_at_next_delimiter() -> None:
    """An oversized frame must not prevent the next valid frame."""
    node = Stm32Node(lambda _: None)
    oversized = bytes([1]) * (MAX_COBS_SIZE + 2) + b'\x00'
    valid = encode_uart_frame(motion_message(14).encode())
    node.input_wire(oversized + valid)
    assert [message.seq for message in node.received] == [14]
    assert node.uart.rejected_frames == 1


def test_current_v1_drop_has_no_retry() -> None:
    """Record that current V1 does not recover a dropped frame."""
    rig = ThreeNodeRig()
    rig.linux_send(motion_message(15))
    rig.stm32_wire_chunks.clear()
    rig.flush_linux_to_stm32()
    assert rig.stm32.received == []


def test_current_v1_duplicate_is_executed_twice() -> None:
    """Record the current lack of request-ID duplicate suppression."""
    node = Stm32Node(lambda _: None)
    wire = encode_uart_frame(motion_message(16).encode())
    node.input_wire(wire + wire)
    assert node.execution_count == 2


def test_esp32_local_ping_returns_pong() -> None:
    """A Linux PING addressed to ESP32 must return a matching PONG."""
    rig = ThreeNodeRig()
    rig.linux_send(
        ApplicationMessage(
            flags=MessageFlag.NONE,
            src=NodeId.LINUX,
            dst=NodeId.ESP32,
            service=ServiceId.SYSTEM,
            opcode=SystemOpcode.PING,
            seq=17,
        )
    )
    assert len(rig.linux_received) == 1
    assert rig.linux_received[0].opcode == SystemOpcode.PONG
    assert rig.linux_received[0].seq == 17


def test_linux_can_directly_use_same_uart_v1_packet() -> None:
    """The V1 packet must remain usable without the ESP32 gateway."""
    node = Stm32Node(lambda _: None)
    packet = motion_message(18).encode()
    node.input_wire(encode_uart_frame(packet))
    assert node.received[0].encode() == packet
