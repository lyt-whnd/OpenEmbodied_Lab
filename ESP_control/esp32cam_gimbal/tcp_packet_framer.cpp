#include "tcp_packet_framer.h"


namespace TcpPacketFraming
{

bool encodeLengthPrefix(
    size_t length,
    uint8_t output[LENGTH_PREFIX_SIZE]
)
{
    if (
        output == nullptr ||
        length == 0U ||
        length > TransportLimits::MAX_OPAQUE_FRAME_SIZE ||
        length > 0xFFFFU
    )
    {
        return false;
    }

    output[0] = static_cast<uint8_t>(
        length & 0xFFU
    );
    output[1] = static_cast<uint8_t>(
        (length >> 8U) & 0xFFU
    );
    return true;
}


StreamDecoder::StreamDecoder()
{
    reset();
}


void StreamDecoder::reset()
{
    lengthPrefixLength_ = 0U;
    expectedLength_ = 0U;
    frameLength_ = 0U;
}


PushStatus StreamDecoder::push(
    uint8_t value,
    FrameView &frame
)
{
    frame = {};

    if (lengthPrefixLength_ < LENGTH_PREFIX_SIZE)
    {
        lengthPrefix_[
            lengthPrefixLength_
        ] = value;
        lengthPrefixLength_++;

        if (lengthPrefixLength_ < LENGTH_PREFIX_SIZE)
        {
            return PushStatus::NEED_MORE_DATA;
        }

        expectedLength_ =
            static_cast<size_t>(lengthPrefix_[0]) |
            (
                static_cast<size_t>(lengthPrefix_[1])
                << 8U
            );

        if (
            expectedLength_ == 0U ||
            expectedLength_ >
                TransportLimits::MAX_OPAQUE_FRAME_SIZE
        )
        {
            reset();
            return PushStatus::INVALID_LENGTH;
        }

        return PushStatus::NEED_MORE_DATA;
    }

    frameBuffer_[frameLength_] = value;
    frameLength_++;

    if (frameLength_ < expectedLength_)
    {
        return PushStatus::NEED_MORE_DATA;
    }

    frame.data = frameBuffer_;
    frame.length = frameLength_;

    lengthPrefixLength_ = 0U;
    expectedLength_ = 0U;
    frameLength_ = 0U;

    return PushStatus::FRAME_READY;
}

}
