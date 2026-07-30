"""Decode and dispatch fixed-header high-rate SAMPLE_BLOCK payloads."""

import struct
from dataclasses import dataclass
from typing import Callable, Dict, List, Tuple


SAMPLE_BLOCK_HEADER_SIZE = 18
SAMPLE_BLOCK_MAX_BYTES = 256
_HEADER = struct.Struct('<HBBIIHHH')
SampleKey = Tuple[int, int, int]


@dataclass(frozen=True)
class SampleBlock:
    sensor_type: int
    instance_id: int
    schema_version: int
    first_timestamp_ms: int
    sample_period_ms: int
    sample_size: int
    drop_count: int
    samples: Tuple[bytes, ...]

    @property
    def key(self) -> SampleKey:
        return (
            self.sensor_type,
            self.instance_id,
            self.schema_version,
        )

    @property
    def timestamps_ms(self) -> Tuple[int, ...]:
        return tuple(
            (
                self.first_timestamp_ms
                + index * self.sample_period_ms
            ) & 0xFFFFFFFF
            for index in range(len(self.samples))
        )


def decode_sample_block(payload: bytes) -> SampleBlock:
    data = bytes(payload)
    if (
        len(data) < _HEADER.size
        or len(data) > SAMPLE_BLOCK_MAX_BYTES
    ):
        raise ValueError('invalid SAMPLE_BLOCK size')
    (
        sensor_type,
        instance_id,
        schema_version,
        first_timestamp_ms,
        sample_period_ms,
        sample_count,
        sample_size,
        drop_count,
    ) = _HEADER.unpack_from(data)
    if sample_count == 0 or sample_size == 0 or sample_period_ms == 0:
        raise ValueError('invalid SAMPLE_BLOCK count/size/period')
    expected = _HEADER.size + sample_count * sample_size
    if len(data) != expected:
        raise ValueError('SAMPLE_BLOCK count/length mismatch')
    samples: List[bytes] = []
    offset = _HEADER.size
    for _ in range(sample_count):
        samples.append(data[offset:offset + sample_size])
        offset += sample_size
    return SampleBlock(
        sensor_type=sensor_type,
        instance_id=instance_id,
        schema_version=schema_version,
        first_timestamp_ms=first_timestamp_ms,
        sample_period_ms=sample_period_ms,
        sample_size=sample_size,
        drop_count=drop_count,
        samples=tuple(samples),
    )


SampleHandler = Callable[[SampleBlock], None]


class SampleBlockDispatcher:
    def __init__(self):
        self._handlers: Dict[SampleKey, SampleHandler] = {}
        self.unknown_schema_count = 0
        self.dispatched_count = 0

    def register(self, key: SampleKey, handler: SampleHandler) -> None:
        if key in self._handlers:
            raise ValueError(f'duplicate sample handler: {key}')
        self._handlers[key] = handler

    def dispatch(self, payload: bytes) -> bool:
        block = decode_sample_block(payload)
        handler = self._handlers.get(block.key)
        if handler is None:
            self.unknown_schema_count += 1
            return False
        handler(block)
        self.dispatched_count += 1
        return True
