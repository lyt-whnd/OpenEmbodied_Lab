"""Robot application protocol V1 codec and motion payload helpers."""

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag


PROTOCOL_VERSION = 1
APPLICATION_HEADER_SIZE = 10
MAX_PAYLOAD_SIZE = 256

_APPLICATION_HEADER = struct.Struct('<BBBBBBHH')
_MOTION_MOVE_PAYLOAD = struct.Struct('<HHhhhh')


class ProtocolError(ValueError):
    """Raised when an application message is malformed."""


class MessageFlag(IntFlag):
    """Application message flags."""

    NONE = 0
    ACK_REQUIRED = 1 << 0
    RESPONSE = 1 << 1
    ERROR = 1 << 2
    REALTIME = 1 << 3


KNOWN_FLAGS_MASK = int(
    MessageFlag.ACK_REQUIRED
    | MessageFlag.RESPONSE
    | MessageFlag.ERROR
    | MessageFlag.REALTIME
)


class NodeId(IntEnum):
    """Robot node addresses."""

    LINUX = 0x01
    ESP32 = 0x02
    STM32 = 0x03
    BROADCAST = 0xFF


class ServiceId(IntEnum):
    """Application service identifiers."""

    SYSTEM = 0x01
    MOTION = 0x10
    TELEMETRY = 0x20
    CONFIG = 0x30
    EVENT = 0x40
    OTA = 0x50


class SystemOpcode(IntEnum):
    """System service operation identifiers."""

    PING = 0x01
    PONG = 0x02


class MotionOpcode(IntEnum):
    """Motion service operation identifiers."""

    MOVE = 0x01
    STOP = 0x02
    CENTER = 0x03
    ESTOP = 0x04
    CLEAR_ESTOP = 0x05
    STATE = 0x10


def _validate_integer(
    name: str,
    value: int,
    minimum: int,
    maximum: int,
) -> None:
    if not isinstance(value, int):
        raise ProtocolError(f'{name} must be an integer')

    if value < minimum or value > maximum:
        raise ProtocolError(
            f'{name} must be in [{minimum}, {maximum}]'
        )


@dataclass(frozen=True)
class ApplicationMessage:
    """Transport-independent V1 application message."""

    flags: int
    src: int
    dst: int
    service: int
    opcode: int
    seq: int
    payload: bytes = b''
    version: int = PROTOCOL_VERSION

    def encode(self) -> bytes:
        """Encode this message into one application message byte string."""
        _validate_integer('version', self.version, 0, 0xFF)
        _validate_integer('flags', self.flags, 0, 0xFF)
        _validate_integer('src', self.src, 0, 0xFF)
        _validate_integer('dst', self.dst, 0, 0xFF)
        _validate_integer('service', self.service, 0, 0xFF)
        _validate_integer('opcode', self.opcode, 0, 0xFF)
        _validate_integer('seq', self.seq, 0, 0xFFFF)

        if self.version != PROTOCOL_VERSION:
            raise ProtocolError(
                f'unsupported protocol version: {self.version}'
            )

        if self.flags & ~KNOWN_FLAGS_MASK:
            raise ProtocolError(
                f'unknown message flags: 0x{self.flags:02x}'
            )

        if not isinstance(
            self.payload,
            (bytes, bytearray, memoryview),
        ):
            raise ProtocolError('payload must be bytes-like')

        payload = bytes(self.payload)

        if len(payload) > MAX_PAYLOAD_SIZE:
            raise ProtocolError(
                f'payload exceeds {MAX_PAYLOAD_SIZE} bytes'
            )

        header = _APPLICATION_HEADER.pack(
            self.version,
            self.flags,
            self.src,
            self.dst,
            self.service,
            self.opcode,
            self.seq,
            len(payload),
        )

        return header + payload

    @classmethod
    def decode(cls, packet: bytes) -> 'ApplicationMessage':
        """Decode and validate one complete application message."""
        if not isinstance(
            packet,
            (bytes, bytearray, memoryview),
        ):
            raise ProtocolError('packet must be bytes-like')

        packet = bytes(packet)

        if len(packet) < APPLICATION_HEADER_SIZE:
            raise ProtocolError('packet is shorter than the V1 header')

        (
            version,
            flags,
            src,
            dst,
            service,
            opcode,
            seq,
            payload_len,
        ) = _APPLICATION_HEADER.unpack_from(packet)

        if version != PROTOCOL_VERSION:
            raise ProtocolError(
                f'unsupported protocol version: {version}'
            )

        if flags & ~KNOWN_FLAGS_MASK:
            raise ProtocolError(
                f'unknown message flags: 0x{flags:02x}'
            )

        if payload_len > MAX_PAYLOAD_SIZE:
            raise ProtocolError(
                f'payload exceeds {MAX_PAYLOAD_SIZE} bytes'
            )

        expected_length = APPLICATION_HEADER_SIZE + payload_len

        if len(packet) != expected_length:
            raise ProtocolError(
                'packet length does not match payload_len'
            )

        return cls(
            version=version,
            flags=flags,
            src=src,
            dst=dst,
            service=service,
            opcode=opcode,
            seq=seq,
            payload=packet[APPLICATION_HEADER_SIZE:],
        )


@dataclass(frozen=True)
class MotionMovePayload:
    """Velocity setpoint carried by ``MOTION/MOVE``."""

    control_epoch: int
    valid_ms: int
    linear_mm_s: int
    angular_mrad_s: int
    head_yaw_rate_x10: int
    head_pitch_rate_x10: int

    def encode(self) -> bytes:
        """Encode the fixed 12-byte motion payload."""
        _validate_integer(
            'control_epoch',
            self.control_epoch,
            0,
            0xFFFF,
        )
        _validate_integer('valid_ms', self.valid_ms, 1, 0xFFFF)

        for name, value in (
            ('linear_mm_s', self.linear_mm_s),
            ('angular_mrad_s', self.angular_mrad_s),
            ('head_yaw_rate_x10', self.head_yaw_rate_x10),
            ('head_pitch_rate_x10', self.head_pitch_rate_x10),
        ):
            _validate_integer(name, value, -0x8000, 0x7FFF)

        return _MOTION_MOVE_PAYLOAD.pack(
            self.control_epoch,
            self.valid_ms,
            self.linear_mm_s,
            self.angular_mrad_s,
            self.head_yaw_rate_x10,
            self.head_pitch_rate_x10,
        )

    @classmethod
    def decode(cls, payload: bytes) -> 'MotionMovePayload':
        """Decode one fixed 12-byte motion payload."""
        if len(payload) != _MOTION_MOVE_PAYLOAD.size:
            raise ProtocolError(
                'MOTION/MOVE payload must be exactly 12 bytes'
            )

        return cls(*_MOTION_MOVE_PAYLOAD.unpack(payload))


class SequenceGenerator:
    """Generate wrapping unsigned 16-bit message sequence numbers."""

    def __init__(self, initial: int = 0):
        """Initialize the generator with the first value to return."""
        _validate_integer('initial', initial, 0, 0xFFFF)
        self._next_value = initial

    def next_seq(self) -> int:
        """Return the next sequence number and advance the generator."""
        value = self._next_value
        self._next_value = (self._next_value + 1) & 0xFFFF
        return value
