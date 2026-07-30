# flake8: noqa
"""Generated from protocol_schema/services.json; do not edit."""

from vision_demo.service_registry import (
    OverflowPolicy,
    Priority,
    QosClass,
)

GENERATED_POLICY_ROWS = (
    (1, 1, QosClass.RELIABLE, Priority.HIGH, 1000, OverflowPolicy.REJECT_NEW),
    (1, 2, QosClass.BEST_EFFORT, Priority.NORMAL, 1000, OverflowPolicy.DROP_OLD),
    (1, 3, QosClass.RELIABLE, Priority.EMERGENCY, 1000, OverflowPolicy.REJECT_NEW),
    (16, 1, QosClass.BEST_EFFORT, Priority.HIGH, 100, OverflowPolicy.DROP_OLD),
    (16, 2, QosClass.RELIABLE, Priority.EMERGENCY, 250, OverflowPolicy.REJECT_NEW),
    (16, 3, QosClass.RELIABLE, Priority.HIGH, 500, OverflowPolicy.REJECT_NEW),
    (16, 4, QosClass.RELIABLE, Priority.EMERGENCY, 100, OverflowPolicy.REJECT_NEW),
    (16, 5, QosClass.RELIABLE, Priority.HIGH, 500, OverflowPolicy.REJECT_NEW),
    (16, 16, QosClass.BEST_EFFORT, Priority.NORMAL, 200, OverflowPolicy.DROP_OLD),
    (32, 1, QosClass.BEST_EFFORT, Priority.NORMAL, 500, OverflowPolicy.DROP_OLD),
    (32, 2, QosClass.BEST_EFFORT, Priority.NORMAL, 100, OverflowPolicy.DROP_OLD),
    (32, 3, QosClass.BEST_EFFORT, Priority.HIGH, 50, OverflowPolicy.DROP_OLD),
    (48, 1, QosClass.RELIABLE, Priority.NORMAL, 1000, OverflowPolicy.REJECT_NEW),
    (48, 2, QosClass.RELIABLE, Priority.HIGH, 1000, OverflowPolicy.REJECT_NEW),
    (64, 1, QosClass.RELIABLE, Priority.EMERGENCY, 1000, OverflowPolicy.REJECT_NEW),
    (80, 1, QosClass.RELIABLE, Priority.HIGH, 2000, OverflowPolicy.REJECT_NEW),
    (80, 2, QosClass.BULK, Priority.BULK, 5000, OverflowPolicy.PAUSE),
    (80, 3, QosClass.RELIABLE, Priority.HIGH, 5000, OverflowPolicy.REJECT_NEW),
    (80, 4, QosClass.RELIABLE, Priority.EMERGENCY, 1000, OverflowPolicy.REJECT_NEW),
    (96, 1, QosClass.RELIABLE, Priority.NORMAL, 1000, OverflowPolicy.REJECT_NEW),
    (96, 2, QosClass.RELIABLE, Priority.NORMAL, 1000, OverflowPolicy.REJECT_NEW),
    (96, 3, QosClass.RELIABLE, Priority.HIGH, 1000, OverflowPolicy.REJECT_NEW),
    (96, 4, QosClass.RELIABLE, Priority.HIGH, 1000, OverflowPolicy.REJECT_NEW),
    (96, 5, QosClass.RELIABLE, Priority.HIGH, 1000, OverflowPolicy.REJECT_NEW),
    (96, 6, QosClass.BEST_EFFORT, Priority.NORMAL, 500, OverflowPolicy.DROP_OLD),
    (96, 7, QosClass.BEST_EFFORT, Priority.HIGH, 1000, OverflowPolicy.DROP_OLD),
)
