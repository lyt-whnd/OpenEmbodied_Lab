#pragma once

#include <stddef.h>
#include <stdint.h>


using MessageRouterSendCallback = bool (*)(
    const uint8_t *data,
    size_t length
);


/*
 * Bind the two outgoing links. The router owns neither transport.
 */
void messageRouterInit(
    MessageRouterSendCallback networkSend,
    MessageRouterSendCallback stm32Send
);


/*
 * Decode one message from its external trust boundary and route the original
 * bytes unchanged. Each function performs exactly one V1 decode.
 */
bool messageRouterOnNetworkMessage(
    const uint8_t *data,
    size_t length
);


bool messageRouterOnStm32Message(
    const uint8_t *data,
    size_t length
);
