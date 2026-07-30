#pragma once

#include <stddef.h>
#include <stdint.h>

#include "uart_framing.h"


namespace UartStreamFramer
{

enum class PushStatus : uint8_t
{
    NONE = 0,
    FRAME_READY,
    FRAME_TOO_LONG,
    RESYNCHRONIZED
};


struct FrameView
{
    const uint8_t *data;
    size_t length;
};


/*
 * Assemble non-zero COBS bytes until the 0x00 delimiter.
 *
 * Storage is fixed at compile time. After overflow, bytes are discarded until
 * the next delimiter so the following frame starts from a known boundary.
 */
class Receiver
{
public:
    Receiver();

    void reset();

    PushStatus push(
        uint8_t value,
        FrameView &frame
    );

private:
    uint8_t buffer_[
        UartFraming::MAX_COBS_FRAME_SIZE
    ];
    size_t length_;
    bool discardUntilDelimiter_;
};

}
