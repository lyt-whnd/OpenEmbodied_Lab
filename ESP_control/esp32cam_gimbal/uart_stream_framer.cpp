#include "uart_stream_framer.h"


namespace UartStreamFramer
{

Receiver::Receiver()
{
    reset();
}


void Receiver::reset()
{
    length_ = 0;
    discardUntilDelimiter_ = false;
}


PushStatus Receiver::push(
    uint8_t value,
    FrameView &frame
)
{
    frame = {};

    if (value == 0U)
    {
        if (discardUntilDelimiter_)
        {
            discardUntilDelimiter_ = false;
            length_ = 0;
            return PushStatus::RESYNCHRONIZED;
        }

        if (length_ == 0U)
        {
            return PushStatus::NONE;
        }

        frame.data = buffer_;
        frame.length = length_;
        length_ = 0;
        return PushStatus::FRAME_READY;
    }

    if (discardUntilDelimiter_)
    {
        return PushStatus::NONE;
    }

    if (length_ >= sizeof(buffer_))
    {
        length_ = 0;
        discardUntilDelimiter_ = true;
        return PushStatus::FRAME_TOO_LONG;
    }

    buffer_[length_] = value;
    length_++;

    return PushStatus::NONE;
}

}
