#include "system_service.h"


bool systemServiceHandle(
    const ProtocolV1::MessageView &message,
    SystemServiceSendCallback sendCallback
)
{
    if (
        message.service !=
            ProtocolV1::SERVICE_SYSTEM ||
        message.opcode !=
            ProtocolV1::SYSTEM_PING ||
        sendCallback == nullptr
    )
    {
        return false;
    }

    ProtocolV1::MessageView pong = {};

    pong.version = ProtocolV1::VERSION;
    pong.flags = ProtocolV1::FLAG_RESPONSE;
    pong.src = ProtocolV1::NODE_ESP32;
    pong.dst = message.src;
    pong.service = ProtocolV1::SERVICE_SYSTEM;
    pong.opcode = ProtocolV1::SYSTEM_PONG;
    pong.seq = message.seq;
    pong.payloadLength = 0;
    pong.payload = nullptr;

    uint8_t response[
        ProtocolV1::HEADER_SIZE
    ] = {};
    size_t responseLength = 0;

    if (
        !ProtocolV1::encodeMessage(
            pong,
            response,
            sizeof(response),
            responseLength
        )
    )
    {
        return false;
    }

    return sendCallback(
        response,
        responseLength
    );
}
