#include "websocket_service.h"

#include <Arduino.h>

#include "message_router.h"
#include "network_tx_queue.h"
#include "protocol_v1.h"
#include "stm32_uart.h"
#include "tcp_transport.h"
#include "transport_limits.h"
#include "websocket_transport.h"


static_assert(
    ProtocolV1::MAX_MESSAGE_SIZE <=
        TransportLimits::MAX_OPAQUE_FRAME_SIZE,
    "Protocol V1 exceeds the bounded transport frame"
);


namespace
{

bool enqueueNetworkMessage(
    const uint8_t *data,
    size_t length
)
{
    return networkTxEnqueue(
        data,
        length
    );
}

uint32_t routerClockMs()
{
    return millis();
}


void receiveNetworkMessage(
    const uint8_t *data,
    size_t length
)
{
    if (
        !messageRouterOnNetworkMessage(
            data,
            length
        )
    )
    {
        Serial.println(
            "Network V1 message was rejected "
            "or unsupported"
        );
    }
}


void receiveTcpMessage(
    const uint8_t *data,
    size_t length
)
{
    /*
     * WebSocket remains authoritative when both clients are connected.
     * TCP is a fallback, not a second simultaneous controller.
     */
    if (websocketTransportHasClient())
    {
        return;
    }

    receiveNetworkMessage(
        data,
        length
    );
}


bool networkTransportHasClient()
{
    return (
        websocketTransportHasClient() ||
        tcpTransportHasClient()
    );
}


bool networkTransportSend(
    const uint8_t *data,
    size_t length
)
{
    if (
        websocketTransportHasClient() &&
        websocketTransportSend(data, length)
    )
    {
        return true;
    }

    return tcpTransportSend(
        data,
        length
    );
}


void receiveStm32Message(
    const uint8_t *data,
    size_t length
)
{
    if (
        !messageRouterOnStm32Message(
            data,
            length
        )
    )
    {
        Serial.println(
            "STM32 V1 message was rejected, "
            "unsupported, or could not be forwarded"
        );
    }
}

}


bool websocketServiceRegister(
    httpd_handle_t server
)
{
    networkTxInit(
        networkTransportSend,
        networkTransportHasClient,
        routerClockMs
    );
    messageRouterInit(
        enqueueNetworkMessage,
        stm32TransportSend,
        routerClockMs
    );
    websocketTransportSetReceiveCallback(
        receiveNetworkMessage
    );
    tcpTransportSetReceiveCallback(
        receiveTcpMessage
    );
    stm32TransportSetReceiveCallback(
        receiveStm32Message
    );

    const bool tcpReady =
        tcpTransportBegin();
    const bool websocketReady =
        websocketTransportRegister(server);

    return tcpReady || websocketReady;
}


bool websocketServiceHasClient()
{
    return networkTransportHasClient();
}


void websocketServicePoll()
{
    tcpTransportPoll();
    networkTxPoll();
}
