#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "message_router.h"
#include "protocol_v1.h"


namespace
{

std::vector<uint8_t> networkOutput;
std::vector<uint8_t> stm32Output;
unsigned int networkSendCount = 0;
unsigned int stm32SendCount = 0;


bool captureNetwork(
    const uint8_t *data,
    size_t length
)
{
    networkOutput.assign(data, data + length);
    networkSendCount++;
    return true;
}


bool captureStm32(
    const uint8_t *data,
    size_t length
)
{
    stm32Output.assign(data, data + length);
    stm32SendCount++;
    return true;
}


void resetCapture()
{
    networkOutput.clear();
    stm32Output.clear();
    networkSendCount = 0;
    stm32SendCount = 0;
}


std::vector<uint8_t> encode(
    uint8_t flags,
    uint8_t source,
    uint8_t destination,
    uint8_t service,
    uint8_t opcode,
    uint16_t sequence
)
{
    ProtocolV1::MessageView message = {};

    message.version = ProtocolV1::VERSION;
    message.flags = flags;
    message.src = source;
    message.dst = destination;
    message.service = service;
    message.opcode = opcode;
    message.seq = sequence;
    message.payloadLength = 0;
    message.payload = nullptr;

    std::vector<uint8_t> packet(
        ProtocolV1::MAX_MESSAGE_SIZE
    );
    size_t length = 0;

    assert(
        ProtocolV1::encodeMessage(
            message,
            packet.data(),
            packet.size(),
            length
        )
    );

    packet.resize(length);
    return packet;
}


void testLinuxPingIsHandledLocally()
{
    resetCapture();

    const std::vector<uint8_t> ping = encode(
        0,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::NODE_ESP32,
        ProtocolV1::SERVICE_SYSTEM,
        ProtocolV1::SYSTEM_PING,
        17
    );

    assert(
        messageRouterOnNetworkMessage(
            ping.data(),
            ping.size()
        )
    );
    assert(networkSendCount == 1);
    assert(stm32SendCount == 0);

    ProtocolV1::MessageView pong = {};

    assert(
        ProtocolV1::decodeMessage(
            networkOutput.data(),
            networkOutput.size(),
            pong
        ) == ProtocolV1::DecodeStatus::OK
    );
    assert(pong.src == ProtocolV1::NODE_ESP32);
    assert(pong.dst == ProtocolV1::NODE_LINUX);
    assert(pong.flags == ProtocolV1::FLAG_RESPONSE);
    assert(pong.service == ProtocolV1::SERVICE_SYSTEM);
    assert(pong.opcode == ProtocolV1::SYSTEM_PONG);
    assert(pong.seq == 17);
}


void testNetworkToStm32PreservesOriginalBytes()
{
    resetCapture();

    const std::vector<uint8_t> command = encode(
        ProtocolV1::FLAG_ACK_REQUIRED,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::NODE_STM32,
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_STOP,
        18
    );

    assert(
        messageRouterOnNetworkMessage(
            command.data(),
            command.size()
        )
    );
    assert(stm32SendCount == 1);
    assert(networkSendCount == 0);
    assert(stm32Output == command);
}


void testStm32ToNetworkPreservesOriginalBytes()
{
    resetCapture();

    const std::vector<uint8_t> response = encode(
        ProtocolV1::FLAG_RESPONSE,
        ProtocolV1::NODE_STM32,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_STATE,
        19
    );

    assert(
        messageRouterOnStm32Message(
            response.data(),
            response.size()
        )
    );
    assert(networkSendCount == 1);
    assert(stm32SendCount == 0);
    assert(networkOutput == response);
}


void testBroadcastUsesBothPaths()
{
    resetCapture();

    const std::vector<uint8_t> ping = encode(
        0,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::NODE_BROADCAST,
        ProtocolV1::SERVICE_SYSTEM,
        ProtocolV1::SYSTEM_PING,
        20
    );

    assert(
        messageRouterOnNetworkMessage(
            ping.data(),
            ping.size()
        )
    );
    assert(networkSendCount == 1);
    assert(stm32SendCount == 1);
    assert(stm32Output == ping);

    ProtocolV1::MessageView pong = {};

    assert(
        ProtocolV1::decodeMessage(
            networkOutput.data(),
            networkOutput.size(),
            pong
        ) == ProtocolV1::DecodeStatus::OK
    );
    assert(pong.src == ProtocolV1::NODE_ESP32);
    assert(pong.dst == ProtocolV1::NODE_LINUX);
}


void testInvalidBoundaryMessagesAreRejected()
{
    resetCapture();

    std::vector<uint8_t> invalidSource = encode(
        0,
        ProtocolV1::NODE_STM32,
        ProtocolV1::NODE_ESP32,
        ProtocolV1::SERVICE_SYSTEM,
        ProtocolV1::SYSTEM_PING,
        21
    );

    assert(
        !messageRouterOnNetworkMessage(
            invalidSource.data(),
            invalidSource.size()
        )
    );
    assert(
        !messageRouterOnNetworkMessage(
            invalidSource.data(),
            1
        )
    );

    const std::vector<uint8_t> invalidDestination = encode(
        0,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::NODE_LINUX,
        ProtocolV1::SERVICE_SYSTEM,
        ProtocolV1::SYSTEM_PING,
        22
    );

    assert(
        !messageRouterOnNetworkMessage(
            invalidDestination.data(),
            invalidDestination.size()
        )
    );
    assert(networkSendCount == 0);
    assert(stm32SendCount == 0);
}

}


int main()
{
    messageRouterInit(
        captureNetwork,
        captureStm32
    );

    testLinuxPingIsHandledLocally();
    testNetworkToStm32PreservesOriginalBytes();
    testStm32ToNetworkPreservesOriginalBytes();
    testBroadcastUsesBothPaths();
    testInvalidBoundaryMessagesAreRejected();

    return 0;
}
