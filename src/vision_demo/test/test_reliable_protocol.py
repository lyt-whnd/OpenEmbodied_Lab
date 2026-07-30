"""QoS registry and Linux RELIABLE sender fault-injection tests."""

import json
from pathlib import Path

import pytest

from vision_demo.protocol_v1 import (
    ApplicationMessage,
    EventOpcode,
    MessageFlag,
    MotionOpcode,
    NodeId,
    OtaOpcode,
    ServiceId,
    StatusCode,
)
from vision_demo.reliable_protocol import (
    ReliableRequest,
    ReliableResult,
    ResultStage,
)
from vision_demo.reliable_sender import ReliableSender
from vision_demo.service_registry import (
    OverflowPolicy,
    Priority,
    QosClass,
    lookup_policy,
    registered_policy_count,
    require_policy,
)


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def _result_message(request_packet, stage, status=StatusCode.OK):
    request = ApplicationMessage.decode(request_packet)
    reliable = ReliableRequest.decode(request.payload)
    flags = MessageFlag.RESPONSE

    if stage is ResultStage.FAILED or status != StatusCode.OK:
        flags |= MessageFlag.ERROR

    return ApplicationMessage(
        flags=int(flags),
        src=request.dst,
        dst=request.src,
        service=request.service,
        opcode=request.opcode,
        seq=request.seq,
        payload=ReliableResult(
            epoch=reliable.epoch,
            request_id=reliable.request_id,
            stage=stage,
            status=int(status),
        ).encode(),
    )


def test_generated_registry_matches_canonical_schema():
    root = Path(__file__).resolve().parents[3]
    schema = json.loads(
        (root / 'protocol_schema/services.json').read_text()
    )
    expected_count = sum(
        len(service['opcodes'])
        for service in schema['services']
    )

    assert registered_policy_count() == expected_count

    for service in schema['services']:
        for opcode in service['opcodes']:
            policy = require_policy(service['id'], opcode['id'])
            assert policy.qos.name == opcode['qos']
            assert policy.priority.name == opcode['priority']
            assert policy.deadline_ms == opcode['deadline_ms']
            assert policy.overflow.name == opcode['overflow']


def test_reliable_payload_matches_cross_language_vector():
    root = Path(__file__).resolve().parents[3]
    vector = json.loads(
        (
            root / 'protocol_schema/reliable_vectors.json'
        ).read_text()
    )
    request = vector['request']
    result = vector['result']

    assert ReliableRequest(
        epoch=request['epoch'],
        request_id=request['request_id'],
        payload=bytes.fromhex(request['application_payload_hex']),
    ).encode() == bytes.fromhex(request['encoded_hex'])
    assert ReliableResult(
        epoch=result['epoch'],
        request_id=result['request_id'],
        stage=ResultStage(result['stage']),
        status=result['status'],
    ).encode() == bytes.fromhex(result['encoded_hex'])


def test_required_qos_policies_and_unknown_rejection():
    move = require_policy(ServiceId.MOTION, MotionOpcode.MOVE)
    stop = require_policy(ServiceId.MOTION, MotionOpcode.STOP)
    state = require_policy(ServiceId.MOTION, MotionOpcode.STATE)
    event = require_policy(ServiceId.EVENT, EventOpcode.REPORT)
    ota_chunk = require_policy(ServiceId.OTA, OtaOpcode.CHUNK)

    assert move.qos is QosClass.BEST_EFFORT
    assert move.overflow is OverflowPolicy.DROP_OLD
    assert stop.qos is QosClass.RELIABLE
    assert stop.priority is Priority.EMERGENCY
    assert state.qos is QosClass.BEST_EFFORT
    assert event.qos is QosClass.RELIABLE
    assert ota_chunk.qos is QosClass.BULK
    assert ota_chunk.overflow is OverflowPolicy.PAUSE
    assert lookup_policy(ServiceId.MOTION, 0x7F) is None

    with pytest.raises(ValueError):
        require_policy(ServiceId.MOTION, 0x7F)


def test_ack_loss_retries_the_identical_request():
    clock = _Clock()
    packets = []
    sender = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=7,
        clock=clock,
        retry_timeout_sec=0.1,
        max_retries=2,
    )

    key = sender.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    )

    assert key == (7, 0)
    assert len(packets) == 1

    clock.now = 0.11
    sender.poll()

    assert packets == [packets[0], packets[0]]
    assert sender.retry_manager.retry_count == 1


def test_received_then_failed_resolves_without_more_retries():
    clock = _Clock()
    packets = []
    observed = []
    sender = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=8,
        clock=clock,
        result_callback=lambda message, result: observed.append(
            (message, result)
        ),
    )
    sender.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.CENTER,
    )

    received = _result_message(
        packets[0],
        ResultStage.RECEIVED,
    )
    sender.handle_message(received, received.encode())
    assert sender.retry_manager.pending_count == 1

    failed = _result_message(
        packets[0],
        ResultStage.FAILED,
        StatusCode.ESTOP_ACTIVE,
    )
    sender.handle_message(failed, failed.encode())

    assert sender.retry_manager.pending_count == 0
    assert observed[-1][1].stage is ResultStage.FAILED
    assert observed[-1][1].status == StatusCode.ESTOP_ACTIVE


def test_epoch_separates_restart_identity_and_pending_is_bounded():
    packets = []
    first = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=10,
        capacity=1,
    )
    second = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=11,
        capacity=1,
    )

    assert first.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    ) == (10, 0)
    assert first.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    ) is None
    assert second.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    ) == (11, 0)


def test_retry_limit_expires_pending_request():
    clock = _Clock()
    packets = []
    sender = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=12,
        clock=clock,
        retry_timeout_sec=0.1,
        max_retries=2,
    )
    sender.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    )

    for now in (0.11, 0.22, 0.33):
        clock.now = now
        sender.poll()

    assert len(packets) == 3
    assert sender.retry_manager.retry_count == 2
    assert sender.retry_manager.timeout_count == 1
    assert sender.retry_manager.pending_count == 0


def test_malformed_result_does_not_resolve_request():
    packets = []
    sender = ReliableSender(
        lambda packet: packets.append(packet) or True,
        epoch=13,
    )
    sender.send(
        dst=NodeId.STM32,
        service=ServiceId.MOTION,
        opcode=MotionOpcode.STOP,
    )
    malformed = _result_message(
        packets[0],
        ResultStage.FAILED,
        StatusCode.BUSY,
    )
    malformed = ApplicationMessage(
        flags=int(MessageFlag.RESPONSE),
        src=malformed.src,
        dst=malformed.dst,
        service=malformed.service,
        opcode=malformed.opcode,
        seq=malformed.seq,
        payload=malformed.payload,
    )

    sender.handle_message(malformed, malformed.encode())

    assert sender.invalid_results == 1
    assert sender.retry_manager.pending_count == 1
