#include <assert.h>
#include <stdint.h>

#include <algorithm>
#include <vector>

#include "protocol_v1.h"
#include "reliable_endpoint.h"
#include "service_registry.h"


namespace
{

std::vector<uint8_t> sentPacket;


bool captureSend(const uint8_t *data, size_t length)
{
    sentPacket.assign(data, data + length);
    return true;
}


void writeU16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>(value >> 8U);
}


void writeU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>(value >> 8U);
    data[2] = static_cast<uint8_t>(value >> 16U);
    data[3] = static_cast<uint8_t>(value >> 24U);
}


ProtocolV1::MessageView request(
    uint8_t *payload,
    uint16_t epoch,
    uint32_t requestId,
    uint8_t opcode = ProtocolV1::SYSTEM_PING
)
{
    payload[0] = Reliable::SCHEMA_VERSION;
    writeU16(payload + 1U, epoch);
    writeU32(payload + 3U, requestId);

    ProtocolV1::MessageView message = {};
    message.version = ProtocolV1::VERSION;
    message.flags = ProtocolV1::FLAG_ACK_REQUIRED;
    message.src = ProtocolV1::NODE_LINUX;
    message.dst = ProtocolV1::NODE_ESP32;
    message.service = ProtocolV1::SERVICE_SYSTEM;
    message.opcode = opcode;
    message.seq = 9U;
    message.payloadLength =
        Reliable::REQUEST_HEADER_SIZE;
    message.payload = payload;
    return message;
}


void testPolicyRegistry()
{
    using namespace ServiceRegistry;

    const MessagePolicy *move = lookup(
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_MOVE
    );
    const MessagePolicy *stop = lookup(
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_STOP
    );
    const MessagePolicy *ota = lookup(
        ProtocolV1::SERVICE_OTA,
        ProtocolV1::OTA_CHUNK
    );

    assert(count() == 26U);
    assert(move != nullptr);
    assert(move->qos == QosClass::BEST_EFFORT);
    assert(move->overflow == OverflowPolicy::DROP_OLD);
    assert(stop != nullptr);
    assert(stop->qos == QosClass::RELIABLE);
    assert(stop->priority == Priority::EMERGENCY);
    assert(ota != nullptr);
    assert(ota->qos == QosClass::BULK);
    assert(
        lookup(
            ProtocolV1::SERVICE_SENSOR,
            ProtocolV1::SENSOR_DATA
        )->qos == QosClass::BEST_EFFORT
    );
    assert(lookup(ProtocolV1::SERVICE_MOTION, 0x7FU) == nullptr);
}


void testDuplicateEpochExpiryAndCollision()
{
    Reliable::Endpoint endpoint;
    uint8_t payload[Reliable::REQUEST_HEADER_SIZE] = {};
    ProtocolV1::MessageView message =
        request(payload, 5U, 100U);
    Reliable::RequestView decoded = {};
    Reliable::CachedResult cached = {};

    assert(
        endpoint.begin(
            message,
            10U,
            decoded,
            cached
        ) == Reliable::BeginStatus::NEW_REQUEST
    );
    endpoint.complete(
        message,
        decoded,
        Reliable::ResultStage::APPLIED,
        ProtocolV1::STATUS_OK,
        10U
    );

    assert(
        endpoint.begin(
            message,
            11U,
            decoded,
            cached
        ) == Reliable::BeginStatus::DUPLICATE
    );
    assert(cached.stage == Reliable::ResultStage::APPLIED);

    ProtocolV1::MessageView collision = message;
    collision.opcode = ProtocolV1::SYSTEM_RESET;
    assert(
        endpoint.begin(
            collision,
            12U,
            decoded,
            cached
        ) == Reliable::BeginStatus::INVALID
    );
    assert(
        cached.status ==
        ProtocolV1::STATUS_REQUEST_ID_CONFLICT
    );

    ProtocolV1::MessageView nextEpoch =
        request(payload, 6U, 100U);
    assert(
        endpoint.begin(
            nextEpoch,
            13U,
            decoded,
            cached
        ) == Reliable::BeginStatus::NEW_REQUEST
    );

    message = request(payload, 5U, 100U);
    assert(
        endpoint.begin(
            message,
            11U + Reliable::RESULT_CACHE_TTL_MS,
            decoded,
            cached
        ) == Reliable::BeginStatus::NEW_REQUEST
    );
}


void testResultWireFormat()
{
    Reliable::Endpoint endpoint;
    uint8_t payload[Reliable::REQUEST_HEADER_SIZE] = {};
    ProtocolV1::MessageView message =
        request(payload, 0x1234U, 0x89ABCDEFUL);
    Reliable::RequestView decoded = {};
    Reliable::CachedResult cached = {};

    assert(
        endpoint.begin(
            message,
            0U,
            decoded,
            cached
        ) == Reliable::BeginStatus::NEW_REQUEST
    );
    assert(
        endpoint.sendResult(
            message,
            decoded,
            Reliable::ResultStage::FAILED,
            ProtocolV1::STATUS_NOT_IMPLEMENTED,
            captureSend
        )
    );

    ProtocolV1::MessageView result = {};
    assert(
        ProtocolV1::decodeMessage(
            sentPacket.data(),
            sentPacket.size(),
            result
        ) == ProtocolV1::DecodeStatus::OK
    );
    assert(
        result.flags ==
        (
            ProtocolV1::FLAG_RESPONSE |
            ProtocolV1::FLAG_ERROR
        )
    );
    assert(result.payloadLength == Reliable::RESULT_PAYLOAD_SIZE);
    assert(result.payload[0] == Reliable::SCHEMA_VERSION);
    assert(
        ProtocolV1::readUint16Le(result.payload + 1U) ==
        0x1234U
    );
    assert(result.payload[7] == 3U);
    assert(
        ProtocolV1::readUint16Le(result.payload + 8U) ==
        ProtocolV1::STATUS_NOT_IMPLEMENTED
    );

    const uint8_t expected[] =
    {
        1U, 0x34U, 0x12U,
        0xEFU, 0xCDU, 0xABU, 0x89U,
        3U, 13U, 0U
    };
    assert(sizeof(expected) == result.payloadLength);
    assert(
        std::equal(
            expected,
            expected + sizeof(expected),
            result.payload
        )
    );
}

void testResultCacheEvictsOldestAtFixedCapacity()
{
    Reliable::Endpoint endpoint;
    uint8_t payload[Reliable::REQUEST_HEADER_SIZE] = {};
    Reliable::RequestView decoded = {};
    Reliable::CachedResult cached = {};

    for (
        uint32_t requestId = 0U;
        requestId <= Reliable::RESULT_CACHE_CAPACITY;
        ++requestId
    )
    {
        ProtocolV1::MessageView message =
            request(payload, 1U, requestId);
        assert(
            endpoint.begin(
                message,
                requestId,
                decoded,
                cached
            ) == Reliable::BeginStatus::NEW_REQUEST
        );
        endpoint.complete(
            message,
            decoded,
            Reliable::ResultStage::APPLIED,
            ProtocolV1::STATUS_OK,
            requestId
        );
    }

    ProtocolV1::MessageView oldest =
        request(payload, 1U, 0U);
    assert(
        endpoint.begin(
            oldest,
            20U,
            decoded,
            cached
        ) == Reliable::BeginStatus::NEW_REQUEST
    );
}

}


int main()
{
    testPolicyRegistry();
    testDuplicateEpochExpiryAndCollision();
    testResultWireFormat();
    testResultCacheEvictsOldestAtFixedCapacity();
    return 0;
}
