"""High-rate block reconstruction, dispatch, and topic separation."""

import struct

import pytest

from vision_demo.sample_block import decode_sample_block
from vision_demo.sensor_topic_bridge import SensorTopicBridge


def _block(period=10, count=3, sample_size=2, data=b'\x01\x02' * 3):
    return struct.pack(
        '<HBBIIHHH',
        2,
        0,
        1,
        0xFFFFFFF8,
        period,
        count,
        sample_size,
        4,
    ) + data


def test_time_reconstruction_wrap_and_period_variants():
    block = decode_sample_block(_block())
    assert block.timestamps_ms == (
        0xFFFFFFF8,
        2,
        12,
    )
    assert block.drop_count == 4

    fast = decode_sample_block(_block(period=2))
    assert fast.timestamps_ms == (
        0xFFFFFFF8,
        0xFFFFFFFA,
        0xFFFFFFFC,
    )


def test_count_length_mismatch_is_rejected():
    with pytest.raises(ValueError):
        decode_sample_block(_block(count=4))
    with pytest.raises(ValueError):
        decode_sample_block(_block(sample_size=0))


def test_state_and_sample_data_use_independent_topics():
    published = []
    bridge = SensorTopicBridge(
        lambda topic, data, timestamp: published.append(
            (topic, data, timestamp)
        )
    )
    bridge.register_samples(2, 0, 1)
    bridge.register_state(1, 0, 1)
    state = (
        b'\x01\x01'
        + struct.pack('<HBBIHH', 1, 0, 1, 50, 2, 1)
        + b'\xaa'
    )
    assert bridge.telemetry.dispatch_batch(
        state,
        local_receive_time=1.0,
    ) == 1
    assert bridge.samples.dispatch(_block())
    assert {item[0] for item in published} == {
        '/robot/sensors/0001/0',
        '/robot/samples/0002/0',
    }
