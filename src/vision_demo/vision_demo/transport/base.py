"""Transport contract for one full application packet at a time."""

from abc import ABC, abstractmethod
from typing import Optional


class Transport(ABC):
    """Move complete binary packets without interpreting V1 fields."""

    @abstractmethod
    def connect(self) -> None:
        """Establish the transport connection."""

    @abstractmethod
    def send(self, packet: bytes) -> None:
        """Send one complete binary application packet."""

    @abstractmethod
    def receive(self) -> Optional[bytes]:
        """Return one packet, or None when no packet is currently ready."""

    @abstractmethod
    def close(self) -> None:
        """Close the transport and release its resources."""
