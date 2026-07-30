"""SensorRecord and TELEMETRY_BATCH decoding with per-schema dispatch."""

import struct
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from vision_demo.clock_mapper import ClockMapper, ClockObservation


BATCH_SCHEMA_VERSION = 1
BATCH_MAX_BYTES = 256
_BATCH_HEADER = struct.Struct('<BB')
_RECORD_HEADER = struct.Struct('<HBBIHH')
SensorKey = Tuple[int, int, int]


@dataclass(frozen=True)
class SensorRecord:
    sensor_type: int
    instance_id: int
    schema_version: int
    timestamp_ms: int
    sample_seq: int
    data: bytes

    @property
    def key(self) -> SensorKey:
        return (
            self.sensor_type,
            self.instance_id,
            self.schema_version,
        )


@dataclass(frozen=True)
class ReceivedSensorRecord:
    record: SensorRecord
    clock: ClockObservation


def decode_telemetry_batch(payload: bytes) -> List[SensorRecord]:
    data = bytes(payload)
    if len(data) < _BATCH_HEADER.size:
        raise ValueError('telemetry batch is too short')
    if len(data) > BATCH_MAX_BYTES:
        raise ValueError('telemetry batch exceeds fixed maximum')

    version, count = _BATCH_HEADER.unpack_from(data)
    if version != BATCH_SCHEMA_VERSION:
        raise ValueError('unsupported telemetry batch schema')

    records: List[SensorRecord] = []
    offset = _BATCH_HEADER.size
    for _ in range(count):
        if len(data) - offset < _RECORD_HEADER.size:
            raise ValueError('truncated SensorRecord header')
        (
            sensor_type,
            instance_id,
            schema_version,
            timestamp_ms,
            sample_seq,
            data_length,
        ) = _RECORD_HEADER.unpack_from(data, offset)
        offset += _RECORD_HEADER.size
        end = offset + data_length
        if end > len(data):
            raise ValueError('SensorRecord dataLength exceeds batch')
        records.append(
            SensorRecord(
                sensor_type=sensor_type,
                instance_id=instance_id,
                schema_version=schema_version,
                timestamp_ms=timestamp_ms,
                sample_seq=sample_seq,
                data=data[offset:end],
            )
        )
        offset = end

    if offset != len(data):
        raise ValueError('telemetry batch has trailing bytes')
    return records


RecordHandler = Callable[[ReceivedSensorRecord], None]


class TelemetryDispatcher:
    """Dispatch known schemas while safely skipping unknown records."""

    def __init__(self):
        self._handlers: Dict[SensorKey, RecordHandler] = {}
        self._clocks: Dict[Tuple[int, int], ClockMapper] = {}
        self.unknown_schema_count = 0
        self.dispatched_count = 0

    def register(self, key: SensorKey, handler: RecordHandler) -> None:
        if key in self._handlers:
            raise ValueError(f'duplicate sensor schema handler: {key}')
        self._handlers[key] = handler

    def reset_clocks(self) -> None:
        for mapper in self._clocks.values():
            mapper.reset()

    def dispatch_batch(
        self,
        payload: bytes,
        *,
        local_receive_time: float,
    ) -> int:
        dispatched = 0
        for record in decode_telemetry_batch(payload):
            handler: Optional[RecordHandler] = self._handlers.get(
                record.key
            )
            if handler is None:
                self.unknown_schema_count += 1
                continue
            clock_key = (record.sensor_type, record.instance_id)
            mapper = self._clocks.setdefault(
                clock_key,
                ClockMapper(),
            )
            observation = mapper.observe(
                record.timestamp_ms,
                local_receive_time,
            )
            handler(ReceivedSensorRecord(record, observation))
            dispatched += 1
            self.dispatched_count += 1
        return dispatched
