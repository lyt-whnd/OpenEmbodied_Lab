#include <assert.h>
#include <stdint.h>
#include <string.h>

#include <iostream>

#include "protocol_v1.h"
#include "uart_framing.h"


namespace
{

const uint8_t LINUX_MOVE_MESSAGE[] = {
    0x01, 0x08, 0x01, 0x03, 0x10,
    0x01, 0x34, 0x12, 0x0C, 0x00,
    0x02, 0x00, 0x2C, 0x01, 0x96,
    0x00, 0xD4, 0xFE, 0xC8, 0x00,
    0x00, 0x00
};


void testKnownLinuxMessage()
{
    ProtocolV1::MessageView message = {};

    const ProtocolV1::DecodeStatus status =
        ProtocolV1::decodeMessage(
            LINUX_MOVE_MESSAGE,
            sizeof(LINUX_MOVE_MESSAGE),
            message
        );

    assert(status == ProtocolV1::DecodeStatus::OK);
    assert(message.version == 1U);
    assert(message.flags == ProtocolV1::FLAG_REALTIME);
    assert(message.src == ProtocolV1::NODE_LINUX);
    assert(message.dst == ProtocolV1::NODE_STM32);
    assert(message.service == ProtocolV1::SERVICE_MOTION);
    assert(message.opcode == ProtocolV1::MOTION_MOVE);
    assert(message.seq == 0x1234U);
    assert(message.payloadLength == 12U);
    assert(message.payload[0] == 0x02U);
    assert(message.payload[7] == 0xFEU);
}


void testPongEncoding()
{
    const uint8_t expected[] = {
        0x01, 0x02, 0x02, 0x01, 0x01,
        0x02, 0x34, 0x12, 0x00, 0x00
    };

    ProtocolV1::MessageView pong = {};

    pong.version = ProtocolV1::VERSION;
    pong.flags = ProtocolV1::FLAG_RESPONSE;
    pong.src = ProtocolV1::NODE_ESP32;
    pong.dst = ProtocolV1::NODE_LINUX;
    pong.service = ProtocolV1::SERVICE_SYSTEM;
    pong.opcode = ProtocolV1::SYSTEM_PONG;
    pong.seq = 0x1234U;
    pong.payloadLength = 0;
    pong.payload = nullptr;

    uint8_t encoded[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};

    size_t encodedLength = 0;

    assert(
        ProtocolV1::encodeMessage(
            pong,
            encoded,
            sizeof(encoded),
            encodedLength
        )
    );
    assert(encodedLength == sizeof(expected));
    assert(
        memcmp(
            encoded,
            expected,
            sizeof(expected)
        ) == 0
    );
}


void testCrcReferenceVector()
{
    static const uint8_t reference[] = {
        '1', '2', '3', '4', '5',
        '6', '7', '8', '9'
    };

    assert(
        UartFraming::crc16CcittFalse(
            reference,
            sizeof(reference)
        ) == 0x29B1U
    );
}


void testUartFrameRoundTrip()
{
    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};

    size_t wireLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            LINUX_MOVE_MESSAGE,
            sizeof(LINUX_MOVE_MESSAGE),
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    );
    assert(wireLength > sizeof(LINUX_MOVE_MESSAGE));
    assert(wireFrame[wireLength - 1U] == 0U);

    uint8_t decoded[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};

    size_t decodedLength = 0;

    const UartFraming::DecodeStatus status =
        UartFraming::decodeApplicationFrame(
            wireFrame,
            wireLength - 1U,
            decoded,
            sizeof(decoded),
            decodedLength
        );

    assert(status == UartFraming::DecodeStatus::OK);
    assert(decodedLength == sizeof(LINUX_MOVE_MESSAGE));
    assert(
        memcmp(
            decoded,
            LINUX_MOVE_MESSAGE,
            decodedLength
        ) == 0
    );

    wireFrame[3] ^= 0x01U;

    const UartFraming::DecodeStatus corruptStatus =
        UartFraming::decodeApplicationFrame(
            wireFrame,
            wireLength - 1U,
            decoded,
            sizeof(decoded),
            decodedLength
        );

    assert(corruptStatus != UartFraming::DecodeStatus::OK);
}


void testMaximumSizeFrame()
{
    uint8_t message[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};

    message[0] = ProtocolV1::VERSION;
    message[1] = ProtocolV1::FLAG_REALTIME;
    message[2] = ProtocolV1::NODE_LINUX;
    message[3] = ProtocolV1::NODE_STM32;
    message[4] = ProtocolV1::SERVICE_OTA;
    message[5] = 0x02U;

    ProtocolV1::writeUint16Le(
        message + 6,
        0xFFFFU
    );
    ProtocolV1::writeUint16Le(
        message + 8,
        ProtocolV1::MAX_PAYLOAD_SIZE
    );

    for (
        size_t index = ProtocolV1::HEADER_SIZE;
        index < sizeof(message);
        index++
    )
    {
        message[index] =
            static_cast<uint8_t>(index);
    }

    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};

    size_t wireLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            message,
            sizeof(message),
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    );
    assert(
        wireLength <=
        UartFraming::MAX_WIRE_FRAME_SIZE
    );

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
        ) ==
        UartFraming::DecodeStatus::OK
    );
    assert(decodedLength == sizeof(message));
    assert(
        memcmp(
            decoded,
            message,
            sizeof(message)
        ) == 0
    );
}

}


int main()
{
    testKnownLinuxMessage();
    testPongEncoding();
    testCrcReferenceVector();
    testUartFrameRoundTrip();
    testMaximumSizeFrame();

    std::cout
        << "ESP32 protocol V1 host tests passed"
        << std::endl;

    return 0;
}
