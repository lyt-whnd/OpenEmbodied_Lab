#pragma once

#include <stddef.h>
#include <stdint.h>

#include "transport_limits.h"


namespace TcpPacketFraming
{

static constexpr size_t LENGTH_PREFIX_SIZE = 2U;

struct FrameView
{
    const uint8_t *data;
    size_t length;
};

enum class PushStatus
{
    NEED_MORE_DATA,
    FRAME_READY,
    INVALID_LENGTH
};

bool encodeLengthPrefix(
    size_t length,
    uint8_t output[LENGTH_PREFIX_SIZE]
);


class StreamDecoder
{
public:
    StreamDecoder();

    void reset();

    PushStatus push(
        uint8_t value,
        FrameView &frame
    );

private:
    uint8_t lengthPrefix_[LENGTH_PREFIX_SIZE];
    size_t lengthPrefixLength_;
    uint8_t frameBuffer_[
        TransportLimits::MAX_OPAQUE_FRAME_SIZE
    ];
    size_t expectedLength_;
    size_t frameLength_;
};

}
