#include "message_router.h"

#include "protocol_v1.h"
#include "reliable_endpoint.h"
#include "service_registry.h"
#include "system_service.h"


namespace
{

MessageRouterSendCallback sendToNetwork = nullptr;
MessageRouterSendCallback sendToStm32 = nullptr;
MessageRouterClockCallback readClock = nullptr;
Reliable::Endpoint reliableEndpoint;


uint32_t currentTimeMs()
{
    return readClock == nullptr ? 0U : readClock();
}


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
    MessageRouterSendCallback stm32Send,
    MessageRouterClockCallback clockNow
)
{
    sendToNetwork = networkSend;
    sendToStm32 = stm32Send;
    readClock = clockNow;
    reliableEndpoint.reset();
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

    const ServiceRegistry::MessagePolicy *policy =
        ServiceRegistry::lookup(
            message.service,
            message.opcode
        );

    if (policy == nullptr)
    {
        return false;
    }

    bool handled = false;

    if (
        message.dst == ProtocolV1::NODE_ESP32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        const bool reliableRequest = (
            policy->qos ==
                ServiceRegistry::QosClass::RELIABLE &&
            (
                message.flags &
                ProtocolV1::FLAG_ACK_REQUIRED
            ) != 0U &&
            (
                message.flags &
                ProtocolV1::FLAG_RESPONSE
            ) == 0U
        );

        if (reliableRequest)
        {
            Reliable::RequestView request = {};
            Reliable::CachedResult cached = {};
            const Reliable::BeginStatus beginStatus =
                reliableEndpoint.begin(
                    message,
                    currentTimeMs(),
                    request,
                    cached
                );

            if (
                beginStatus ==
                Reliable::BeginStatus::DUPLICATE
            )
            {
                handled = reliableEndpoint.sendResult(
                    message,
                    request,
                    cached.stage,
                    cached.status,
                    sendToNetwork
                ) || handled;
            }
            else if (
                beginStatus ==
                    Reliable::BeginStatus::INVALID &&
                cached.status != 0U
            )
            {
                handled = reliableEndpoint.sendResult(
                    message,
                    request,
                    Reliable::ResultStage::FAILED,
                    cached.status,
                    sendToNetwork
                ) || handled;
            }
            else if (
                beginStatus ==
                Reliable::BeginStatus::NEW_REQUEST
            )
            {
                (void)reliableEndpoint.sendResult(
                    message,
                    request,
                    Reliable::ResultStage::RECEIVED,
                    ProtocolV1::STATUS_OK,
                    sendToNetwork
                );

                ProtocolV1::MessageView application =
                    message;
                application.payload = request.payload;
                application.payloadLength =
                    request.payloadLength;

                const bool applied =
                    systemServiceHandle(
                        application,
                        sendToNetwork
                    );
                const Reliable::ResultStage stage = (
                    applied
                    ? Reliable::ResultStage::APPLIED
                    : Reliable::ResultStage::FAILED
                );
                const uint16_t status = (
                    applied
                    ? ProtocolV1::STATUS_OK
                    : ProtocolV1::STATUS_NOT_IMPLEMENTED
                );

                reliableEndpoint.complete(
                    message,
                    request,
                    stage,
                    status,
                    currentTimeMs()
                );
                handled = reliableEndpoint.sendResult(
                    message,
                    request,
                    stage,
                    status,
                    sendToNetwork
                ) || handled;
            }
        }
        else
        {
            handled =
                systemServiceHandle(
                    message,
                    sendToNetwork
                ) ||
                handled;
        }
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

    if (
        ServiceRegistry::lookup(
            message.service,
            message.opcode
        ) == nullptr
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
