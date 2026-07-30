#include "message_router.h"

#include "protocol_v1.h"
#include "system_service.h"


namespace
{

MessageRouterSendCallback sendToNetwork = nullptr;
MessageRouterSendCallback sendToStm32 = nullptr;


bool isNetworkDestinationAllowed(
    uint8_t destination
)
{
    return (
        destination == ProtocolV1::NODE_ESP32 ||
        destination == ProtocolV1::NODE_STM32 ||
        destination == ProtocolV1::NODE_BROADCAST
    );
}


bool isStm32DestinationAllowed(
    uint8_t destination
)
{
    return (
        destination == ProtocolV1::NODE_LINUX ||
        destination == ProtocolV1::NODE_ESP32 ||
        destination == ProtocolV1::NODE_BROADCAST
    );
}

}


void messageRouterInit(
    MessageRouterSendCallback networkSend,
    MessageRouterSendCallback stm32Send
)
{
    sendToNetwork = networkSend;
    sendToStm32 = stm32Send;
}


bool messageRouterOnNetworkMessage(
    const uint8_t *data,
    size_t length
)
{
    ProtocolV1::MessageView message = {};

    if (
        ProtocolV1::decodeMessage(
            data,
            length,
            message
        ) != ProtocolV1::DecodeStatus::OK ||
        message.src != ProtocolV1::NODE_LINUX ||
        !isNetworkDestinationAllowed(message.dst)
    )
    {
        return false;
    }

    bool handled = false;

    if (
        message.dst == ProtocolV1::NODE_ESP32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled =
            systemServiceHandle(
                message,
                sendToNetwork
            ) ||
            handled;
    }

    if (
        message.dst == ProtocolV1::NODE_STM32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled = (
            sendToStm32 != nullptr &&
            sendToStm32(data, length)
        ) || handled;
    }

    return handled;
}


bool messageRouterOnStm32Message(
    const uint8_t *data,
    size_t length
)
{
    ProtocolV1::MessageView message = {};

    if (
        ProtocolV1::decodeMessage(
            data,
            length,
            message
        ) != ProtocolV1::DecodeStatus::OK ||
        message.src != ProtocolV1::NODE_STM32 ||
        !isStm32DestinationAllowed(message.dst)
    )
    {
        return false;
    }

    bool handled = false;

    if (
        message.dst == ProtocolV1::NODE_ESP32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled =
            systemServiceHandle(
                message,
                sendToStm32
            ) ||
            handled;
    }

    if (
        message.dst == ProtocolV1::NODE_LINUX ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled = (
            sendToNetwork != nullptr &&
            sendToNetwork(data, length)
        ) || handled;
    }

    return handled;
}
