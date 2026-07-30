"""Behavior tests for the bounded three-class transmit scheduler."""

from vision_demo.service_registry import (
    MessagePolicy,
    OverflowPolicy,
    Priority,
    QosClass,
)
from vision_demo.tx_scheduler import QueueKind, TxScheduler


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _policy(qos, priority, deadline_ms=100):
    return MessagePolicy(
        qos=qos,
        priority=priority,
        deadline_ms=deadline_ms,
        overflow=OverflowPolicy.DROP_OLD,
    )


def test_bulk_backlog_never_delays_emergency():
    clock = _Clock()
    scheduler = TxScheduler(
        reliable_capacity=2,
        latest_capacity=2,
        sample_capacity=2,
        bulk_capacity=2,
        clock=clock,
    )
    bulk = _policy(QosClass.BULK, Priority.BULK, 0)
    emergency = _policy(
        QosClass.RELIABLE,
        Priority.EMERGENCY,
        250,
    )

    assert scheduler.enqueue(
        b'bulk-1', kind=QueueKind.BULK, policy=bulk, key=(1,)
    )
    assert scheduler.enqueue(
        b'bulk-2', kind=QueueKind.BULK, policy=bulk, key=(2,)
    )
    assert scheduler.enqueue(
        b'stop',
        kind=QueueKind.RELIABLE,
        policy=emergency,
        key=(3,),
    )

    assert scheduler.take_next() == b'stop'
    assert scheduler.take_next() == b'bulk-1'


def test_latest_replaces_instead_of_building_backlog():
    scheduler = TxScheduler()
    latest = _policy(
        QosClass.BEST_EFFORT,
        Priority.HIGH,
    )

    for value in range(10):
        assert scheduler.enqueue(
            bytes((value,)),
            kind=QueueKind.BEST_EFFORT_LATEST,
            policy=latest,
            key=(0x10, 0x01),
        )

    assert scheduler.pending_count == 1
    assert scheduler.take_next() == b'\x09'
    assert scheduler.stats.replaced_latest == 9


def test_expired_best_effort_is_not_sent():
    clock = _Clock()
    scheduler = TxScheduler(clock=clock)
    realtime = _policy(
        QosClass.BEST_EFFORT,
        Priority.HIGH,
        20,
    )
    assert scheduler.enqueue(
        b'old',
        kind=QueueKind.BEST_EFFORT_LATEST,
        policy=realtime,
        key=(1,),
    )

    clock.now = 0.021
    assert scheduler.take_next() is None
    assert scheduler.stats.expired == 1


def test_sample_ring_drops_oldest_and_counts_it():
    scheduler = TxScheduler(sample_capacity=2)
    samples = _policy(
        QosClass.BEST_EFFORT,
        Priority.NORMAL,
        0,
    )
    for packet in (b'a', b'b', b'c'):
        assert scheduler.enqueue(
            packet,
            kind=QueueKind.BEST_EFFORT_SAMPLE,
            policy=samples,
            key=(1,),
        )

    assert scheduler.take_next() == b'b'
    assert scheduler.take_next() == b'c'
    assert scheduler.stats.dropped_sample == 1
