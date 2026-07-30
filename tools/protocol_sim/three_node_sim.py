"""Deterministic host model of the current Linux/ESP32/STM32 V1 path."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Callable

from vision_demo.protocol_v1 import (
    APPLICATION_HEADER_SIZE,
    ApplicationMessage,
    MAX_PAYLOAD_SIZE,
    MessageFlag,
    NodeId,
    ServiceId,
    SystemOpcode,
)
from vision_demo.reliable_protocol import (
    ReliableRequest,
    ReliableResult,
    ResultStage,
)
from vision_demo.service_registry import QosClass, lookup_policy


CRC_SIZE = 2
MAX_MESSAGE_SIZE = APPLICATION_HEADER_SIZE + MAX_PAYLOAD_SIZE
MAX_RAW_SIZE = MAX_MESSAGE_SIZE + CRC_SIZE
MAX_COBS_SIZE = MAX_RAW_SIZE + (MAX_RAW_SIZE // 254) + 1


class FrameError(ValueError):
    """Raised when a simulated UART frame is malformed."""


def crc16_ccitt_false(data: bytes) -> int:
    """Return CRC-16/CCITT-FALSE for *data*."""
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    """Encode one payload with Consistent Overhead Byte Stuffing."""
    output = bytearray(b'\x00')
    code_index = 0
    code = 1

    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
            continue

        output.append(value)
        code += 1
        if code == 0xFF:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    """Decode one delimiter-free COBS payload."""
    if not data:
        raise FrameError('empty COBS frame')

    output = bytearray()
    read_index = 0
    while read_index < len(data):
        code = data[read_index]
        if code == 0:
            raise FrameError('zero byte inside COBS frame')
        read_index += 1
        block_end = read_index + code - 1
        if block_end > len(data):
            raise FrameError('truncated COBS block')
        output.extend(data[read_index:block_end])
        read_index = block_end
        if code != 0xFF and read_index < len(data):
            output.append(0)
    return bytes(output)


def encode_uart_frame(packet: bytes) -> bytes:
    """Wrap one V1 application packet in CRC, COBS and delimiter."""
    crc = crc16_ccitt_false(packet)
    raw = packet + crc.to_bytes(2, 'little')
    return cobs_encode(raw) + b'\x00'


def decode_uart_frame(encoded: bytes) -> bytes:
    """Validate and unwrap one delimiter-free UART frame."""
    raw = cobs_decode(encoded)
    if len(raw) < CRC_SIZE:
        raise FrameError('UART frame is shorter than its CRC')
    packet = raw[:-CRC_SIZE]
    received_crc = int.from_bytes(raw[-CRC_SIZE:], 'little')
    if received_crc != crc16_ccitt_false(packet):
        raise FrameError('UART frame CRC mismatch')
    ApplicationMessage.decode(packet)
    return packet


@dataclass
class UartStreamReceiver:
    """Current delimiter receiver, including oversize-frame recovery."""

    on_packet: Callable[[bytes], None]
    encoded: bytearray = field(default_factory=bytearray)
    dropping_oversized_frame: bool = False
    rejected_frames: int = 0

    def input_bytes(self, data: bytes) -> None:
        """Consume an arbitrary UART byte-stream chunk."""
        for value in data:
            if value == 0:
                if self.dropping_oversized_frame:
                    self.dropping_oversized_frame = False
                    self.encoded.clear()
                    continue
                if self.encoded:
                    try:
                        self.on_packet(decode_uart_frame(bytes(self.encoded)))
                    except FrameError:
                        self.rejected_frames += 1
                    self.encoded.clear()
                continue

            if self.dropping_oversized_frame:
                continue

            if len(self.encoded) >= MAX_COBS_SIZE:
                self.rejected_frames += 1
                self.encoded.clear()
                self.dropping_oversized_frame = True
                continue

            self.encoded.append(value)


@dataclass
class Stm32Node:
    """Small behavioral model of the current STM32 protocol endpoint."""

    send_wire: Callable[[bytes], None]
    received: list[ApplicationMessage] = field(default_factory=list)
    execution_count: int = 0

    def __post_init__(self) -> None:
        """Create the STM32 UART stream receiver."""
        self.uart = UartStreamReceiver(self._on_packet)
        self.result_cache = {}

    def _on_packet(self, packet: bytes) -> None:
        message = ApplicationMessage.decode(packet)
        if message.src not in (NodeId.LINUX, NodeId.ESP32):
            return
        if message.dst not in (NodeId.STM32, NodeId.BROADCAST):
            return

        policy = lookup_policy(message.service, message.opcode)
        is_reliable_request = (
            policy is not None
            and policy.qos is QosClass.RELIABLE
            and bool(message.flags & MessageFlag.ACK_REQUIRED)
            and not bool(message.flags & MessageFlag.RESPONSE)
        )

        if is_reliable_request:
            request = ReliableRequest.decode(message.payload)
            key = (
                message.src,
                request.epoch,
                request.request_id,
            )
            cached = self.result_cache.get(key)

            if cached is not None:
                self._send_result(message, request, cached)
                return

            self._send_result(
                message,
                request,
                ReliableResult(
                    epoch=request.epoch,
                    request_id=request.request_id,
                    stage=ResultStage.RECEIVED,
                    status=0,
                ),
            )
            message = replace(
                message,
                payload=request.payload,
            )
            applied = ReliableResult(
                epoch=request.epoch,
                request_id=request.request_id,
                stage=ResultStage.APPLIED,
                status=0,
            )
            self.result_cache[key] = applied
            self.received.append(message)
            self.execution_count += 1
            self._send_result(message, request, applied)
            return

        self.received.append(message)
        self.execution_count += 1

    def input_wire(self, data: bytes) -> None:
        """Deliver one arbitrary UART byte-stream chunk."""
        self.uart.input_bytes(data)

    def send_message(self, message: ApplicationMessage) -> None:
        """Send one STM32-originated message over UART."""
        self.send_wire(encode_uart_frame(message.encode()))

    def _send_result(
        self,
        request_message: ApplicationMessage,
        _request: ReliableRequest,
        result: ReliableResult,
    ) -> None:
        """Return one generic reliable stage through the UART gateway."""
        flags = MessageFlag.RESPONSE

        if result.stage is ResultStage.FAILED:
            flags |= MessageFlag.ERROR

        self.send_message(
            ApplicationMessage(
                flags=int(flags),
                src=int(NodeId.STM32),
                dst=request_message.src,
                service=request_message.service,
                opcode=request_message.opcode,
                seq=request_message.seq,
                payload=result.encode(),
            )
        )


@dataclass
class Esp32Gateway:
    """Model ESP32 as both a local node and a transparent V1 gateway."""

    send_linux: Callable[[bytes], None]
    send_stm32_wire: Callable[[bytes], None]

    def __post_init__(self) -> None:
        """Create the gateway UART stream receiver."""
        self.uart = UartStreamReceiver(self._on_stm32_packet)

    def input_linux_packet(self, packet: bytes) -> None:
        """Consume one complete WebSocket/TCP application packet."""
        message = ApplicationMessage.decode(packet)
        if message.src != NodeId.LINUX:
            return

        if message.dst in (NodeId.ESP32, NodeId.BROADCAST):
            self._handle_local(message)
        if message.dst in (NodeId.STM32, NodeId.BROADCAST):
            self.send_stm32_wire(encode_uart_frame(packet))

    def input_stm32_wire(self, data: bytes) -> None:
        """Consume an arbitrary UART chunk from STM32."""
        self.uart.input_bytes(data)

    def _on_stm32_packet(self, packet: bytes) -> None:
        message = ApplicationMessage.decode(packet)
        if message.src != NodeId.STM32:
            return
        if message.dst in (NodeId.LINUX, NodeId.BROADCAST):
            self.send_linux(packet)

    def _handle_local(self, message: ApplicationMessage) -> None:
        if (
            message.service == ServiceId.SYSTEM
            and message.opcode == SystemOpcode.PING
        ):
            response = ApplicationMessage(
                flags=MessageFlag.RESPONSE,
                src=NodeId.ESP32,
                dst=NodeId.LINUX,
                service=ServiceId.SYSTEM,
                opcode=SystemOpcode.PONG,
                seq=message.seq,
            )
            self.send_linux(response.encode())


@dataclass
class ThreeNodeRig:
    """Wire together deterministic Linux, ESP32 and STM32 endpoints."""

    linux_received: list[ApplicationMessage] = field(default_factory=list)

    def __post_init__(self) -> None:
        """Connect the simulated nodes through in-memory transports."""
        self.stm32_wire_chunks: list[bytes] = []
        self.esp32 = Esp32Gateway(
            self._on_linux_packet,
            self.stm32_wire_chunks.append,
        )
        self.stm32 = Stm32Node(self.esp32.input_stm32_wire)

    def _on_linux_packet(self, packet: bytes) -> None:
        self.linux_received.append(ApplicationMessage.decode(packet))

    def linux_send(self, message: ApplicationMessage) -> None:
        """Send a complete application packet from Linux to ESP32."""
        self.esp32.input_linux_packet(message.encode())

    def flush_linux_to_stm32(self, chunk_sizes: tuple[int, ...] = ()) -> None:
        """Deliver queued ESP32 UART bytes with optional fragmentation."""
        wire = b''.join(self.stm32_wire_chunks)
        self.stm32_wire_chunks.clear()
        if not chunk_sizes:
            self.stm32.input_wire(wire)
            return

        offset = 0
        for size in chunk_sizes:
            self.stm32.input_wire(wire[offset:offset + size])
            offset += size
        self.stm32.input_wire(wire[offset:])
