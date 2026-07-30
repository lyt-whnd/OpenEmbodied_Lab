"""Primary/fallback transport selection without changing RobotLink."""

from typing import Callable, Optional

from vision_demo.transport.base import Transport


TransportFactory = Callable[[], Transport]


class FallbackTransport(Transport):
    """Prefer one transport and use another when connection fails."""

    def __init__(
        self,
        primary_factory: TransportFactory,
        fallback_factory: TransportFactory,
    ):
        self._primary_factory = primary_factory
        self._fallback_factory = fallback_factory
        self._active: Optional[Transport] = None
        self.active_name: Optional[str] = None

    def connect(self) -> None:
        self.close()
        primary = self._primary_factory()

        try:
            primary.connect()
        except Exception:
            primary.close()
        else:
            self._active = primary
            self.active_name = type(primary).__name__
            return

        fallback = self._fallback_factory()

        try:
            fallback.connect()
        except Exception:
            fallback.close()
            raise

        self._active = fallback
        self.active_name = type(fallback).__name__

    def send(self, packet: bytes) -> None:
        self._require_active().send(packet)

    def receive(self) -> Optional[bytes]:
        return self._require_active().receive()

    def close(self) -> None:
        active = self._active
        self._active = None
        self.active_name = None

        if active is not None:
            active.close()

    def _require_active(self) -> Transport:
        if self._active is None:
            raise ConnectionError(
                'Fallback transport is not connected'
            )
        return self._active
