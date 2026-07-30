#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol_v1.h"


using SystemServiceSendCallback = bool (*)(
    const uint8_t *data,
    size_t length
);


/*
 * Handle one Protocol V1 message addressed to this node.
 *
 * Stage 1 keeps the existing SYSTEM/PING -> SYSTEM/PONG behavior.
 */
bool systemServiceHandle(
    const ProtocolV1::MessageView &message,
    SystemServiceSendCallback sendCallback
);
