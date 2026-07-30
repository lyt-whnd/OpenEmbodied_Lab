"""Wire helpers for bounded RELIABLE request/result messages."""

import struct
from dataclasses import dataclass
from enum import IntEnum

from vision_demo.protocol_v1 import ProtocolError


RELIABLE_SCHEMA_VERSION = 1
_REQUEST_HEADER = struct.Struct('<BHI')
_RESULT_PAYLOAD = struct.Struct('<BHIBH')


class ResultStage(IntEnum):
    """Execution stage reported by a reliable endpoint."""

    RECEIVED = 1
    APPLIED = 2
    FAILED = 3


@dataclass(frozen=True)
class ReliableRequest:
    """Stable identity plus service-specific command payload."""

    epoch: int
    request_id: int
    payload: bytes = b''

    def encode(self) -> bytes:
        """Encode the reliable prefix followed by application payload."""
        if not 0 <= self.epoch <= 0xFFFF:
            raise ProtocolError('epoch must fit uint16')
        if not 0 <= self.request_id <= 0xFFFFFFFF:
            raise ProtocolError('request_id must fit uint32')

        return _REQUEST_HEADER.pack(
            RELIABLE_SCHEMA_VERSION,
            self.epoch,
            self.request_id,
        ) + bytes(self.payload)

    @classmethod
    def decode(cls, payload: bytes) -> 'ReliableRequest':
        """Decode and validate one reliable request payload."""
        if len(payload) < _REQUEST_HEADER.size:
            raise ProtocolError('reliable request header is truncated')

        version, epoch, request_id = _REQUEST_HEADER.unpack_from(
            payload
        )

        if version != RELIABLE_SCHEMA_VERSION:
            raise ProtocolError(
                f'unsupported reliable schema version: {version}'
            )

        return cls(
            epoch=epoch,
            request_id=request_id,
            payload=bytes(payload[_REQUEST_HEADER.size:]),
        )


@dataclass(frozen=True)
class ReliableResult:
    """Generic result correlated by sender epoch and request ID."""

    epoch: int
    request_id: int
    stage: ResultStage
    status: int

    def encode(self) -> bytes:
        """Encode the fixed result payload."""
        if not 0 <= self.epoch <= 0xFFFF:
            raise ProtocolError('epoch must fit uint16')
        if not 0 <= self.request_id <= 0xFFFFFFFF:
            raise ProtocolError('request_id must fit uint32')
        if not 0 <= self.status <= 0xFFFF:
            raise ProtocolError('status must fit uint16')

        try:
            result_stage = ResultStage(self.stage)
        except ValueError as error:
            raise ProtocolError(
                f'unknown reliable result stage: {self.stage}'
            ) from error

        if (
            result_stage in (
                ResultStage.RECEIVED,
                ResultStage.APPLIED,
            )
            and self.status != 0
        ):
            raise ProtocolError(
                'successful reliable stage requires status 0'
            )

        if (
            result_stage is ResultStage.FAILED
            and self.status == 0
        ):
            raise ProtocolError(
                'FAILED reliable stage requires nonzero status'
            )

        return _RESULT_PAYLOAD.pack(
            RELIABLE_SCHEMA_VERSION,
            self.epoch,
            self.request_id,
            int(result_stage),
            self.status,
        )

    @classmethod
    def decode(cls, payload: bytes) -> 'ReliableResult':
        """Decode and validate one fixed result payload."""
        if len(payload) != _RESULT_PAYLOAD.size:
            raise ProtocolError('reliable result must be 10 bytes')

        version, epoch, request_id, stage, status = (
            _RESULT_PAYLOAD.unpack(payload)
        )

        if version != RELIABLE_SCHEMA_VERSION:
            raise ProtocolError(
                f'unsupported reliable schema version: {version}'
            )

        try:
            result_stage = ResultStage(stage)
        except ValueError as error:
            raise ProtocolError(
                f'unknown reliable result stage: {stage}'
            ) from error

        if (
            result_stage in (
                ResultStage.RECEIVED,
                ResultStage.APPLIED,
            )
            and status != 0
        ):
            raise ProtocolError(
                'successful reliable stage requires status 0'
            )

        if result_stage is ResultStage.FAILED and status == 0:
            raise ProtocolError(
                'FAILED reliable stage requires nonzero status'
            )

        return cls(
            epoch=epoch,
            request_id=request_id,
            stage=result_stage,
            status=status,
        )
