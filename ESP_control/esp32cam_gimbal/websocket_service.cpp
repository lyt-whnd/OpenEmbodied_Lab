#include "websocket_service.h"

#include <Arduino.h>

#include "message_router.h"
#include "network_tx_queue.h"
#include "protocol_v1.h"
#include "stm32_uart.h"
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
            "WebSocket V1 message was rejected "
            "or unsupported"
        );
    }
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
        websocketTransportSend,
        websocketTransportHasClient
    );
    messageRouterInit(
        enqueueNetworkMessage,
        stm32TransportSend
    );
    websocketTransportSetReceiveCallback(
        receiveNetworkMessage
    );
    stm32TransportSetReceiveCallback(
        receiveStm32Message
    );

    return websocketTransportRegister(server);
}


bool websocketServiceHasClient()
{
    return websocketTransportHasClient();
}


void websocketServicePoll()
{
    networkTxPoll();
}
