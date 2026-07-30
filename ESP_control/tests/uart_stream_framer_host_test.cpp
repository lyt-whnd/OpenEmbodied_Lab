#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <iostream>

#include "protocol_v1.h"
#include "uart_framing.h"
#include "uart_stream_framer.h"


namespace
{

const uint8_t MESSAGE[] = {
    0x01, 0x08, 0x01, 0x03, 0x10,
    0x01, 0x34, 0x12, 0x02, 0x00,
    0xA5, 0x5A
};


void decodeAndCheck(
    const UartStreamFramer::FrameView &frame
)
{
    uint8_t decoded[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};
    size_t decodedLength = 0;

    assert(
        UartFraming::decodeApplicationFrame(
            frame.data,
            frame.length,
            decoded,
            sizeof(decoded),
            decodedLength
        ) == UartFraming::DecodeStatus::OK
    );
    assert(decodedLength == sizeof(MESSAGE));
    assert(
        memcmp(
            decoded,
            MESSAGE,
            sizeof(MESSAGE)
        ) == 0
    );
}


void testConsecutiveFramesAndEmptyDelimiters()
{
    uint8_t wire[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};
    size_t wireLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            MESSAGE,
            sizeof(MESSAGE),
            wire,
            sizeof(wire),
            wireLength
        )
    );

    UartStreamFramer::Receiver receiver;
    size_t readyCount = 0;

    for (size_t repeat = 0; repeat < 2U; repeat++)
    {
        for (size_t index = 0; index < wireLength; index++)
        {
            UartStreamFramer::FrameView frame = {};
            const auto status =
                receiver.push(wire[index], frame);

            if (
                status ==
                UartStreamFramer::PushStatus::FRAME_READY
            )
            {
                decodeAndCheck(frame);
                readyCount++;
            }
        }
    }

    UartStreamFramer::FrameView emptyFrame = {};

    assert(
        receiver.push(0U, emptyFrame) ==
        UartStreamFramer::PushStatus::NONE
    );
    assert(readyCount == 2U);
}


void testOverflowDiscardAndRecovery()
{
    UartStreamFramer::Receiver receiver;

    for (
        size_t index = 0;
        index < UartFraming::MAX_COBS_FRAME_SIZE;
        index++
    )
    {
        UartStreamFramer::FrameView frame = {};

        assert(
            receiver.push(0x7EU, frame) ==
            UartStreamFramer::PushStatus::NONE
        );
    }

    UartStreamFramer::FrameView overflowFrame = {};

    assert(
        receiver.push(0x7EU, overflowFrame) ==
        UartStreamFramer::PushStatus::FRAME_TOO_LONG
    );
    assert(
        receiver.push(0x7EU, overflowFrame) ==
        UartStreamFramer::PushStatus::NONE
    );
    assert(
        receiver.push(0U, overflowFrame) ==
        UartStreamFramer::PushStatus::RESYNCHRONIZED
    );

    uint8_t wire[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};
    size_t wireLength = 0;

    assert(
        UartFraming::encodeApplicationFrame(
            MESSAGE,
            sizeof(MESSAGE),
            wire,
            sizeof(wire),
            wireLength
        )
    );

    size_t readyCount = 0;

    for (size_t index = 0; index < wireLength; index++)
    {
        UartStreamFramer::FrameView frame = {};
        const auto status =
            receiver.push(wire[index], frame);

        if (
            status ==
            UartStreamFramer::PushStatus::FRAME_READY
        )
        {
            decodeAndCheck(frame);
            readyCount++;
        }
    }

    assert(readyCount == 1U);
}

}


int main()
{
    testConsecutiveFramesAndEmptyDelimiters();
    testOverflowDiscardAndRecovery();

    std::cout
        << "UART stream framer host tests passed"
        << std::endl;

    return 0;
}
