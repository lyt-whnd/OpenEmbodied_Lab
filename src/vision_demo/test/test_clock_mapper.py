"""Wrap, delay, out-of-order, and reconnect tests for ClockMapper."""

import pytest

from vision_demo.clock_mapper import ClockMapper


def test_delayed_receive_keeps_remote_sample_timestamp():
    mapper = ClockMapper(smoothing=1.0)
    first = mapper.observe(1000, 11.0)
    delayed = mapper.observe(1020, 11.5)

    assert first.extended_remote_ms == 1000
    assert delayed.extended_remote_ms == 1020
    assert delayed.local_receive_time == 11.5
    assert delayed.mapped_time == pytest.approx(11.02)


def test_uint32_wrap_extends_monotonically():
    mapper = ClockMapper(smoothing=1.0)
    before = mapper.observe(0xFFFFFFF0, 100.0)
    after = mapper.observe(0x00000010, 100.032)

    assert (
        after.extended_remote_ms
        - before.extended_remote_ms
    ) == 32
    assert mapper.reset_count == 0


def test_out_of_order_and_reconnect_reset_mapping():
    mapper = ClockMapper()
    mapper.observe(1000, 5.0)
    restarted = mapper.observe(900, 6.0)

    assert mapper.reset_count == 1
    assert restarted.extended_remote_ms == 900
    assert restarted.mapped_time == pytest.approx(6.0)

    mapper.reset()
    with pytest.raises(RuntimeError):
        mapper.map_remote(901)
