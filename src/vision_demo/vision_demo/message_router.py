"""Decode each received V1 packet once and dispatch it to handlers."""

from collections import defaultdict
from typing import Callable, DefaultDict, List, Tuple

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    ProtocolError,
)
from vision_demo.service_registry import lookup_policy


MessageHandler = Callable[[ApplicationMessage, bytes], None]
RouteKey = Tuple[int, int]


class MessageRouter:
    """Route decoded messages by service/opcode and notify observers."""

    def __init__(self):
        """Create an empty, fixed-purpose handler registry."""
        self._handlers: DefaultDict[
            RouteKey,
            List[MessageHandler],
        ] = defaultdict(list)
        self._observers: List[MessageHandler] = []
        self.decode_errors = 0
        self.policy_errors = 0

    def register_handler(
        self,
        service: int,
        opcode: int,
        handler: MessageHandler,
    ) -> None:
        """Register a handler for one service/opcode pair."""
        self._handlers[(int(service), int(opcode))].append(handler)

    def register_observer(self, observer: MessageHandler) -> None:
        """Observe every valid decoded message."""
        self._observers.append(observer)

    def route_packet(self, packet: bytes) -> bool:
        """Decode one packet once and synchronously dispatch it."""
        try:
            message = ApplicationMessage.decode(packet)
        except (ProtocolError, TypeError, ValueError):
            self.decode_errors += 1
            return False

        if lookup_policy(message.service, message.opcode) is None:
            self.policy_errors += 1
            return False

        immutable_packet = bytes(packet)

        for observer in tuple(self._observers):
            observer(message, immutable_packet)

        handlers = self._handlers.get(
            (int(message.service), int(message.opcode)),
            (),
        )

        for handler in tuple(handlers):
            handler(message, immutable_packet)

        return True
