"""Boundary and dispatch tests for SensorRecord batches."""

import struct

import pytest

from vision_demo.telemetry import (
    BATCH_MAX_BYTES,
    TelemetryDispatcher,
    decode_telemetry_batch,
)


def _record(sensor_type, instance, schema, timestamp, seq, data):
    return struct.pack(
        '<HBBIHH',
        sensor_type,
        instance,
        schema,
        timestamp,
        seq,
        len(data),
    ) + data


def test_unknown_schema_skips_without_corrupting_next_record():
    unknown = _record(1, 0, 99, 10, 1, b'abc')
    known = _record(2, 0, 1, 20, 2, b'xyz')
    payload = b'\x01\x02' + unknown + known
    received = []
    dispatcher = TelemetryDispatcher()
    dispatcher.register((2, 0, 1), received.append)

    assert dispatcher.dispatch_batch(
        payload,
        local_receive_time=5.0,
    ) == 1
    assert received[0].record.data == b'xyz'
    assert received[0].record.timestamp_ms == 20
    assert dispatcher.unknown_schema_count == 1


def test_exact_maximum_and_one_byte_over_are_checked():
    data = b'x' * (BATCH_MAX_BYTES - 2 - 12)
    exact = b'\x01\x01' + _record(1, 0, 1, 0, 0, data)
    assert len(exact) == BATCH_MAX_BYTES
    assert decode_telemetry_batch(exact)[0].data == data

    with pytest.raises(ValueError):
        decode_telemetry_batch(exact + b'x')


def test_malformed_data_length_and_trailing_bytes():
    malformed = (
        b'\x01\x01'
        + struct.pack('<HBBIHH', 1, 0, 1, 0, 0, 5)
        + b'ab'
    )
    with pytest.raises(ValueError):
        decode_telemetry_batch(malformed)

    with pytest.raises(ValueError):
        decode_telemetry_batch(b'\x01\x00x')
