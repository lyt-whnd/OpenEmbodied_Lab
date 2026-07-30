#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <iostream>
#include <vector>

#include "tcp_packet_framer.h"


namespace
{

std::vector<uint8_t> framePacket(
    const std::vector<uint8_t> &packet
)
{
    uint8_t prefix[
        TcpPacketFraming::LENGTH_PREFIX_SIZE
    ] = {};
    assert(
        TcpPacketFraming::encodeLengthPrefix(
            packet.size(),
            prefix
        )
    );

    std::vector<uint8_t> result = {
        prefix[0],
        prefix[1]
    };
    result.insert(
        result.end(),
        packet.begin(),
        packet.end()
    );
    return result;
}


void testFragmentedAndCoalescedPackets()
{
    TcpPacketFraming::StreamDecoder decoder;
    const auto first = framePacket(
        {0x01U, 0x02U, 0x03U}
    );
    const auto second = framePacket(
        {0x11U, 0x12U}
    );

    std::vector<uint8_t> stream = first;
    stream.insert(
        stream.end(),
        second.begin(),
        second.end()
    );

    std::vector<std::vector<uint8_t>> frames;

    for (uint8_t value : stream)
    {
        TcpPacketFraming::FrameView frame = {};
        const auto status = decoder.push(
            value,
            frame
        );

        if (
            status ==
            TcpPacketFraming::PushStatus::FRAME_READY
        )
        {
            frames.emplace_back(
                frame.data,
                frame.data + frame.length
            );
        }
    }

    assert(frames.size() == 2U);
    assert(
        frames[0] ==
        std::vector<uint8_t>({0x01U, 0x02U, 0x03U})
    );
    assert(
        frames[1] ==
        std::vector<uint8_t>({0x11U, 0x12U})
    );
}


void testInvalidLengthsResetDecoder()
{
    TcpPacketFraming::StreamDecoder decoder;
    TcpPacketFraming::FrameView frame = {};

    assert(
        decoder.push(0U, frame) ==
        TcpPacketFraming::PushStatus::NEED_MORE_DATA
    );
    assert(
        decoder.push(0U, frame) ==
        TcpPacketFraming::PushStatus::INVALID_LENGTH
    );

    const size_t tooLarge =
        TransportLimits::MAX_OPAQUE_FRAME_SIZE + 1U;
    assert(
        decoder.push(
            static_cast<uint8_t>(
                tooLarge & 0xFFU
            ),
            frame
        ) ==
        TcpPacketFraming::PushStatus::NEED_MORE_DATA
    );
    assert(
        decoder.push(
            static_cast<uint8_t>(
                tooLarge >> 8U
            ),
            frame
        ) ==
        TcpPacketFraming::PushStatus::INVALID_LENGTH
    );

    uint8_t prefix[2] = {};
    assert(!TcpPacketFraming::encodeLengthPrefix(0U, prefix));
    assert(
        !TcpPacketFraming::encodeLengthPrefix(
            tooLarge,
            prefix
        )
    );
}

}


int main()
{
    testFragmentedAndCoalescedPackets();
    testInvalidLengthsResetDecoder();

    std::cout
        << "TCP packet framer host tests passed\n";
    return 0;
}
