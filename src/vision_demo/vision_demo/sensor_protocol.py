"""Transport-independent payload schemas for the generic SENSOR service."""

import struct
from dataclasses import dataclass
from typing import Tuple

from vision_demo.service_registry import QosClass


SENSOR_SCHEMA_VERSION = 1
_TARGET = struct.Struct('<HB')
_LIST_ENTRY = struct.Struct('<HBB')
_INFO = struct.Struct('<BHBBBBHHH')
_CONFIG = struct.Struct('<HBHi')


@dataclass(frozen=True)
class SensorTarget:
    sensor_type: int
    instance_id: int

    def encode(self) -> bytes:
        return _TARGET.pack(self.sensor_type, self.instance_id)


@dataclass(frozen=True)
class SensorListEntry:
    sensor_type: int
    instance_id: int
    schema_version: int


@dataclass(frozen=True)
class SensorInfo:
    sensor_type: int
    instance_id: int
    schema_version: int
    default_qos: QosClass
    buffer_policy: int
    sample_size: int
    sample_period_ms: int
    max_latency_ms: int
    driver_info: bytes


def decode_sensor_list(payload: bytes) -> Tuple[SensorListEntry, ...]:
    data = bytes(payload)
    if len(data) < 2 or data[0] != SENSOR_SCHEMA_VERSION:
        raise ValueError('invalid SENSOR_LIST response')
    count = data[1]
    if len(data) != 2 + count * _LIST_ENTRY.size:
        raise ValueError('SENSOR_LIST length mismatch')
    return tuple(
        SensorListEntry(*_LIST_ENTRY.unpack_from(data, 2 + index * 4))
        for index in range(count)
    )


def decode_sensor_info(payload: bytes) -> SensorInfo:
    data = bytes(payload)
    if len(data) < _INFO.size:
        raise ValueError('SENSOR_INFO response is too short')
    (
        schema,
        sensor_type,
        instance_id,
        data_schema,
        qos,
        buffer_policy,
        sample_size,
        sample_period_ms,
        max_latency_ms,
    ) = _INFO.unpack_from(data)
    if schema != SENSOR_SCHEMA_VERSION:
        raise ValueError('unsupported SENSOR_INFO schema')
    return SensorInfo(
        sensor_type=sensor_type,
        instance_id=instance_id,
        schema_version=data_schema,
        default_qos=QosClass(qos),
        buffer_policy=buffer_policy,
        sample_size=sample_size,
        sample_period_ms=sample_period_ms,
        max_latency_ms=max_latency_ms,
        driver_info=data[_INFO.size:],
    )


def encode_sensor_config(
    target: SensorTarget,
    key: int,
    value: int,
) -> bytes:
    return _CONFIG.pack(
        target.sensor_type,
        target.instance_id,
        key,
        value,
    )
