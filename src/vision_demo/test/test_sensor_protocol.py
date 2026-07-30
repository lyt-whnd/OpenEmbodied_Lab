"""Golden behavior for generic SENSOR service payloads."""

import pytest

from vision_demo.sensor_protocol import (
    SensorTarget,
    decode_sensor_info,
    decode_sensor_list,
    encode_sensor_config,
)
from vision_demo.service_registry import QosClass


def test_sensor_list_and_info_are_generic():
    entries = decode_sensor_list(
        b'\x01\x02'
        b'\x01\x00\x00\x01'
        b'\x02\x00\x03\x04'
    )
    assert entries[1].sensor_type == 2
    assert entries[1].instance_id == 3
    assert entries[1].schema_version == 4

    info = decode_sensor_info(
        b'\x01\x01\x00\x00\x02\x00\x01'
        b'\x04\x00\x0a\x00\x14\x00extra'
    )
    assert info.default_qos is QosClass.BEST_EFFORT
    assert info.sample_size == 4
    assert info.sample_period_ms == 10
    assert info.max_latency_ms == 20
    assert info.driver_info == b'extra'


def test_target_config_and_malformed_length():
    target = SensorTarget(sensor_type=0x1234, instance_id=2)
    assert target.encode() == b'\x34\x12\x02'
    assert encode_sensor_config(target, 7, -5) == (
        b'\x34\x12\x02\x07\x00\xfb\xff\xff\xff'
    )

    with pytest.raises(ValueError):
        decode_sensor_list(b'\x01\x01')
