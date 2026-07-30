"""Transport-independent V1 service and QoS policy registry."""

from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, Optional, Tuple


class QosClass(IntEnum):
    """Bounded delivery behavior selected for one message kind."""

    BEST_EFFORT = 0
    RELIABLE = 1
    BULK = 2


class Priority(IntEnum):
    """Scheduling priority; larger values run first."""

    BULK = 0
    NORMAL = 1
    HIGH = 2
    EMERGENCY = 3


class OverflowPolicy(IntEnum):
    """Action taken when the bounded storage for a class is full."""

    DROP_OLD = 0
    REJECT_NEW = 1
    PAUSE = 2


@dataclass(frozen=True)
class MessagePolicy:
    """Default policy for a registered service/opcode pair."""

    qos: QosClass
    priority: Priority
    deadline_ms: int
    overflow: OverflowPolicy


# Import after defining the enums used by the generated table.
from vision_demo.service_registry_generated import (  # noqa: E402
    GENERATED_POLICY_ROWS,
)


_POLICIES: Dict[Tuple[int, int], MessagePolicy] = {
    (service, opcode): MessagePolicy(
        qos=qos,
        priority=priority,
        deadline_ms=deadline_ms,
        overflow=overflow,
    )
    for (
        service,
        opcode,
        qos,
        priority,
        deadline_ms,
        overflow,
    ) in GENERATED_POLICY_ROWS
}


def lookup_policy(
    service: int,
    opcode: int,
) -> Optional[MessagePolicy]:
    """Return the registered policy or ``None`` for unknown messages."""
    return _POLICIES.get((int(service), int(opcode)))


def require_policy(service: int, opcode: int) -> MessagePolicy:
    """Return a policy and reject unregistered opcodes explicitly."""
    policy = lookup_policy(service, opcode)

    if policy is None:
        raise ValueError(
            f'unregistered V1 service/opcode: '
            f'0x{int(service):02x}/0x{int(opcode):02x}'
        )

    return policy


def registered_policy_count() -> int:
    """Return the fixed number of generated policy entries."""
    return len(_POLICIES)
