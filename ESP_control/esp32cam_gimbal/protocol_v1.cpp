#include "protocol_v1.h"

#include <string.h>


namespace ProtocolV1
{

uint16_t readUint16Le(
    const uint8_t *data
)
{
    return static_cast<uint16_t>(
        static_cast<uint16_t>(data[0]) |
        (
            static_cast<uint16_t>(data[1])
            << 8U
        )
    );
}


void writeUint16Le(
    uint8_t *data,
    uint16_t value
)
{
    data[0] = static_cast<uint8_t>(
        value & 0xFFU
    );
    data[1] = static_cast<uint8_t>(
        value >> 8U
    );
}


DecodeStatus decodeMessage(
    const uint8_t *data,
    size_t length,
    MessageView &message
)
{
    message = {};

    if (data == nullptr)
    {
        return DecodeStatus::NULL_DATA;
    }

    if (length < HEADER_SIZE)
    {
        return DecodeStatus::HEADER_TOO_SHORT;
    }

    const uint8_t version = data[0];
    const uint8_t flags = data[1];
    const uint16_t payloadLength =
        readUint16Le(data + 8);

    if (version != VERSION)
    {
        return DecodeStatus::UNSUPPORTED_VERSION;
    }

    if ((flags & ~KNOWN_FLAGS_MASK) != 0U)
    {
        return DecodeStatus::UNKNOWN_FLAGS;
    }

    if (payloadLength > MAX_PAYLOAD_SIZE)
    {
        return DecodeStatus::PAYLOAD_TOO_LARGE;
    }

    const size_t expectedLength =
        HEADER_SIZE +
        static_cast<size_t>(payloadLength);

    if (length != expectedLength)
    {
        return DecodeStatus::LENGTH_MISMATCH;
    }

    message.version = version;
    message.flags = flags;
    message.src = data[2];
    message.dst = data[3];
    message.service = data[4];
    message.opcode = data[5];
    message.seq = readUint16Le(data + 6);
    message.payloadLength = payloadLength;
    message.payload = data + HEADER_SIZE;

    return DecodeStatus::OK;
}


bool encodeMessage(
    const MessageView &message,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
)
{
    outputLength = 0;

    if (
        output == nullptr ||
        message.version != VERSION ||
        (
            message.flags &
            ~KNOWN_FLAGS_MASK
        ) != 0U ||
        message.payloadLength >
            MAX_PAYLOAD_SIZE ||
        (
            message.payloadLength > 0U &&
            message.payload == nullptr
        )
    )
    {
        return false;
    }

    const size_t messageLength =
        HEADER_SIZE +
        static_cast<size_t>(
            message.payloadLength
        );

    if (outputCapacity < messageLength)
    {
        return false;
    }

    output[0] = message.version;
    output[1] = message.flags;
    output[2] = message.src;
    output[3] = message.dst;
    output[4] = message.service;
    output[5] = message.opcode;

    writeUint16Le(
        output + 6,
        message.seq
    );
    writeUint16Le(
        output + 8,
        message.payloadLength
    );

    if (message.payloadLength > 0U)
    {
        memcpy(
            output + HEADER_SIZE,
            message.payload,
            message.payloadLength
        );
    }

    outputLength = messageLength;

    return true;
}


const char *decodeStatusName(
    DecodeStatus status
)
{
    switch (status)
    {
        case DecodeStatus::OK:
            return "OK";

        case DecodeStatus::NULL_DATA:
            return "NULL_DATA";

        case DecodeStatus::HEADER_TOO_SHORT:
            return "HEADER_TOO_SHORT";

        case DecodeStatus::UNSUPPORTED_VERSION:
            return "UNSUPPORTED_VERSION";

        case DecodeStatus::UNKNOWN_FLAGS:
            return "UNKNOWN_FLAGS";

        case DecodeStatus::PAYLOAD_TOO_LARGE:
            return "PAYLOAD_TOO_LARGE";

        case DecodeStatus::LENGTH_MISMATCH:
            return "LENGTH_MISMATCH";

        default:
            return "UNKNOWN_STATUS";
    }
}

}
