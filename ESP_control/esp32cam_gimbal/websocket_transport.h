#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_http_server.h"


using WebsocketTransportReceiveCallback = void (*)(
    const uint8_t *data,
    size_t length
);


/*
 * Register /ws on an existing HTTP server.
 *
 * The receive callback borrows data only for the duration of the callback.
 * This layer knows WebSocket framing, but does not parse Protocol V1.
 */
bool websocketTransportRegister(
    httpd_handle_t server
);


void websocketTransportSetReceiveCallback(
    WebsocketTransportReceiveCallback callback
);


/*
 * Send one complete binary WebSocket frame to the current Linux client.
 *
 * data only needs to remain valid until this function returns.
 */
bool websocketTransportSend(
    const uint8_t *data,
    size_t length
);


bool websocketTransportHasClient();
