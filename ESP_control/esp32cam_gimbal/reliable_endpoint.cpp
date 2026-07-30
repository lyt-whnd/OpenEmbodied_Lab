#include "reliable_endpoint.h"

namespace Reliable
{

namespace
{

uint16_t readU16(const uint8_t *data)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (
            static_cast<uint16_t>(data[1])
            << 8U
        )
    );
}


uint32_t readU32(const uint8_t *data)
{
    return
        static_cast<uint32_t>(data[0]) |
        (
            static_cast<uint32_t>(data[1])
            << 8U
        ) |
        (
            static_cast<uint32_t>(data[2])
            << 16U
        ) |
        (
            static_cast<uint32_t>(data[3])
            << 24U
        );
}


void writeU16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>(value >> 8U);
}


void writeU32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFU);
    data[1] = static_cast<uint8_t>((value >> 8U) & 0xFFU);
    data[2] = static_cast<uint8_t>((value >> 16U) & 0xFFU);
    data[3] = static_cast<uint8_t>(value >> 24U);
}


}


Endpoint::Endpoint()
{
    reset();
}


void Endpoint::reset()
{
    cache_.reset();
}


BeginStatus Endpoint::begin(
    const ProtocolV1::MessageView &message,
    uint32_t nowMs,
    RequestView &request,
    CachedResult &cached
)
{
    request = {};
    cached = {};

    if (
        message.payload == nullptr ||
        message.payloadLength < REQUEST_HEADER_SIZE ||
        message.payload[0] != SCHEMA_VERSION
    )
    {
        return BeginStatus::INVALID;
    }

    request.epoch = readU16(message.payload + 1U);
    request.requestId = readU32(message.payload + 3U);
    request.payload = message.payload + REQUEST_HEADER_SIZE;
    request.payloadLength = static_cast<uint16_t>(
        message.payloadLength - REQUEST_HEADER_SIZE
    );

    const CacheLookup lookup = cache_.find(
        message.src,
        message.service,
        message.opcode,
        request.epoch,
        request.requestId,
        nowMs,
        cached
    );

    if (lookup == CacheLookup::FOUND)
    {
        return BeginStatus::DUPLICATE;
    }

    if (lookup == CacheLookup::CONFLICT)
    {
        cached.stage = ResultStage::FAILED;
        cached.status =
            ProtocolV1::STATUS_REQUEST_ID_CONFLICT;
        return BeginStatus::INVALID;
    }

    return BeginStatus::NEW_REQUEST;
}


void Endpoint::complete(
    const ProtocolV1::MessageView &message,
    const RequestView &request,
    ResultStage stage,
    uint16_t status,
    uint32_t nowMs
)
{
    cache_.store(
        message.src,
        message.service,
        message.opcode,
        request.epoch,
        request.requestId,
        stage,
        status,
        nowMs
    );
}


bool Endpoint::sendResult(
    const ProtocolV1::MessageView &requestMessage,
    const RequestView &request,
    ResultStage stage,
    uint16_t status,
    SendCallback send
) const
{
    if (send == nullptr)
    {
        return false;
    }

    uint8_t payload[RESULT_PAYLOAD_SIZE] = {};
    payload[0] = SCHEMA_VERSION;
    writeU16(payload + 1U, request.epoch);
    writeU32(payload + 3U, request.requestId);
    payload[7] = static_cast<uint8_t>(stage);
    writeU16(payload + 8U, status);

    ProtocolV1::MessageView result = {};
    result.version = ProtocolV1::VERSION;
    result.flags = ProtocolV1::FLAG_RESPONSE;

    if (stage == ResultStage::FAILED || status != 0U)
    {
        result.flags |= ProtocolV1::FLAG_ERROR;
    }

    result.src = ProtocolV1::NODE_ESP32;
    result.dst = requestMessage.src;
    result.service = requestMessage.service;
    result.opcode = requestMessage.opcode;
    result.seq = requestMessage.seq;
    result.payloadLength = sizeof(payload);
    result.payload = payload;

    uint8_t packet[ProtocolV1::MAX_MESSAGE_SIZE] = {};
    size_t packetLength = 0;

    return (
        ProtocolV1::encodeMessage(
            result,
            packet,
            sizeof(packet),
            packetLength
        ) &&
        send(packet, packetLength)
    );
}

}
