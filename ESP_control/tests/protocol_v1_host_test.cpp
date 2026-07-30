#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <iostream>

#include "protocol_v1.h"
#include "uart_framing.h"
#include "v1_vectors.h"


static_assert(
    ProtocolV1::VERSION == V1_GOLDEN_PROTOCOL_VERSION,
    "protocol version differs from canonical vectors"
);
static_assert(
    ProtocolV1::HEADER_SIZE == V1_GOLDEN_HEADER_SIZE,
    "header size differs from canonical vectors"
);
static_assert(
    ProtocolV1::MAX_PAYLOAD_SIZE ==
        V1_GOLDEN_MAX_PAYLOAD_SIZE,
    "payload limit differs from canonical vectors"
);
static_assert(
    ProtocolV1::KNOWN_FLAGS_MASK ==
        V1_GOLDEN_KNOWN_FLAGS_MASK,
    "flag mask differs from canonical vectors"
);
static_assert(
    ProtocolV1::NODE_LINUX == V1_GOLDEN_NODE_LINUX &&
        ProtocolV1::NODE_ESP32 == V1_GOLDEN_NODE_ESP32 &&
        ProtocolV1::NODE_STM32 == V1_GOLDEN_NODE_STM32 &&
        ProtocolV1::NODE_BROADCAST == V1_GOLDEN_NODE_BROADCAST,
    "node IDs differ from canonical vectors"
);
static_assert(
    ProtocolV1::SERVICE_SYSTEM == V1_GOLDEN_SERVICE_SYSTEM &&
        ProtocolV1::SERVICE_MOTION == V1_GOLDEN_SERVICE_MOTION &&
        ProtocolV1::SERVICE_TELEMETRY ==
            V1_GOLDEN_SERVICE_TELEMETRY &&
        ProtocolV1::SERVICE_CONFIG == V1_GOLDEN_SERVICE_CONFIG &&
        ProtocolV1::SERVICE_EVENT == V1_GOLDEN_SERVICE_EVENT &&
        ProtocolV1::SERVICE_OTA == V1_GOLDEN_SERVICE_OTA,
    "service IDs differ from canonical vectors"
);


namespace
{

const V1GoldenValidVector &findValidVector(
    const char *name
)
{
    for (
        size_t index = 0;
        index < V1_GOLDEN_VALID_VECTOR_COUNT;
        index++
    )
    {
        if (
            strcmp(
                V1_GOLDEN_VALID_VECTORS[index].name,
                name
            ) == 0
        )
        {
            return V1_GOLDEN_VALID_VECTORS[index];
        }
    }

    assert(false);
    return V1_GOLDEN_VALID_VECTORS[0];
}


void testGoldenValidMessages()
{
    for (
        size_t index = 0;
        index < V1_GOLDEN_VALID_VECTOR_COUNT;
        index++
    )
    {
        const V1GoldenValidVector &vector =
            V1_GOLDEN_VALID_VECTORS[index];
        ProtocolV1::MessageView message = {};

        const ProtocolV1::DecodeStatus status =
            ProtocolV1::decodeMessage(
                vector.message,
                vector.message_length,
                message
            );

        assert(status == ProtocolV1::DecodeStatus::OK);
        assert(message.version == vector.version);
        assert(message.flags == vector.flags);
        assert(message.src == vector.src);
        assert(message.dst == vector.dst);
        assert(message.service == vector.service);
        assert(message.opcode == vector.opcode);
        assert(message.seq == vector.seq);
        assert(message.payloadLength == vector.payload_length);
        assert(
            message.payload ==
            vector.message + ProtocolV1::HEADER_SIZE
        );

        uint8_t encoded[
            ProtocolV1::MAX_MESSAGE_SIZE
        ] = {};
        size_t encodedLength = 0;

        assert(
            ProtocolV1::encodeMessage(
                message,
                encoded,
                sizeof(encoded),
                encodedLength
            )
        );
        assert(encodedLength == vector.message_length);
        assert(
            memcmp(
                encoded,
                vector.message,
                vector.message_length
            ) == 0
        );
    }
}


void testGoldenInvalidMessages()
{
    for (
        size_t index = 0;
        index < V1_GOLDEN_INVALID_VECTOR_COUNT;
        index++
    )
    {
        const V1GoldenInvalidVector &vector =
            V1_GOLDEN_INVALID_VECTORS[index];
        ProtocolV1::MessageView message = {};

        const ProtocolV1::DecodeStatus status =
            ProtocolV1::decodeMessage(
                vector.message,
                vector.message_length,
                message
            );

        assert(status != ProtocolV1::DecodeStatus::OK);
        assert(
            strcmp(
                ProtocolV1::decodeStatusName(status),
                vector.expected_error
            ) == 0
        );
    }
}


void testKnownLinuxMessagePayload()
{
    const V1GoldenValidVector &vector =
        findValidVector("linux_motion_move");
    ProtocolV1::MessageView message = {};

    assert(
        ProtocolV1::decodeMessage(
            vector.message,
            vector.message_length,
            message
        ) == ProtocolV1::DecodeStatus::OK
    );
    assert(message.payload[0] == 0x02U);
    assert(message.payload[7] == 0xFEU);
}


void testCrcReferenceVector()
{
    assert(
        UartFraming::crc16CcittFalse(
            V1_GOLDEN_CRC_DATA,
            sizeof(V1_GOLDEN_CRC_DATA)
        ) == V1_GOLDEN_CRC_EXPECTED
    );
}


void testAllGoldenUartFrameRoundTrips()
{
    for (
        size_t index = 0;
        index < V1_GOLDEN_VALID_VECTOR_COUNT;
        index++
    )
    {
        const V1GoldenValidVector &vector =
            V1_GOLDEN_VALID_VECTORS[index];
        uint8_t wireFrame[
            UartFraming::MAX_WIRE_FRAME_SIZE
        ] = {};
        size_t wireLength = 0;

        assert(
            UartFraming::encodeApplicationFrame(
                vector.message,
                vector.message_length,
                wireFrame,
                sizeof(wireFrame),
                wireLength
            )
        );
        assert(wireLength > vector.message_length);
        assert(wireFrame[wireLength - 1U] == 0U);

        uint8_t decoded[
            ProtocolV1::MAX_MESSAGE_SIZE
        ] = {};
        size_t decodedLength = 0;

        assert(
            UartFraming::decodeApplicationFrame(
                wireFrame,
                wireLength - 1U,
                decoded,
                sizeof(decoded),
                decodedLength
            ) == UartFraming::DecodeStatus::OK
        );
        assert(decodedLength == vector.message_length);
        assert(
            memcmp(
                decoded,
                vector.message,
                decodedLength
            ) == 0
        );
    }
}


void testCorruptUartFrameIsRejected()
{
    const V1GoldenValidVector &vector =
        findValidVector("linux_motion_move");
    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};
    size_t wireLength = 0;
    uint8_t decoded[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};
    size_t decodedLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            vector.message,
            vector.message_length,
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    );

    wireFrame[3] ^= 0x01U;

    assert(
        UartFraming::decodeApplicationFrame(
            wireFrame,
            wireLength - 1U,
            decoded,
            sizeof(decoded),
            decodedLength
        ) != UartFraming::DecodeStatus::OK
    );
}


void testFramingBoundaryErrors()
{
    uint8_t message[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};
    size_t messageLength = 0;
    const uint8_t emptyRaw[] = {0x01U};

    assert(
        UartFraming::decodeApplicationFrame(
            emptyRaw,
            sizeof(emptyRaw),
            message,
            sizeof(message),
            messageLength
        ) == UartFraming::DecodeStatus::FRAME_TOO_SHORT
    );

    uint8_t oversized[
        UartFraming::MAX_COBS_FRAME_SIZE + 1U
    ] = {};

    assert(
        UartFraming::decodeApplicationFrame(
            oversized,
            sizeof(oversized),
            message,
            sizeof(message),
            messageLength
        ) == UartFraming::DecodeStatus::INPUT_TOO_LARGE
    );

    const V1GoldenValidVector &vector =
        findValidVector("linux_motion_move");
    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};
    size_t wireLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            vector.message,
            vector.message_length,
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    );
    assert(wireLength > 2U);
    assert(
        UartFraming::decodeApplicationFrame(
            wireFrame,
            wireLength - 2U,
            message,
            sizeof(message),
            messageLength
        ) != UartFraming::DecodeStatus::OK
    );
}


void testNullApplicationDataIsRejected()
{
    ProtocolV1::MessageView message = {};

    assert(
        ProtocolV1::decodeMessage(
            nullptr,
            0,
            message
        ) == ProtocolV1::DecodeStatus::NULL_DATA
    );
}

}


int main()
{
    testGoldenValidMessages();
    testGoldenInvalidMessages();
    testKnownLinuxMessagePayload();
    testCrcReferenceVector();
    testAllGoldenUartFrameRoundTrips();
    testCorruptUartFrameIsRejected();
    testFramingBoundaryErrors();
    testNullApplicationDataIsRejected();

    std::cout
        << "ESP32 protocol V1 host tests passed"
        << std::endl;

    return 0;
}
