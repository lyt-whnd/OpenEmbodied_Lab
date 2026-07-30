#pragma once

#include "esp_http_server.h"


/*
 * Wire WebSocket plus TCP fallback, the bounded network TX queue, V1 router,
 * local services, and STM32 link.
 *
 * This coordinator does not parse WebSocket frames or Protocol V1 itself.
 */
bool websocketServiceRegister(
    httpd_handle_t server
);


/*
 * Return whether WebSocket or TCP owns a live Linux client.
 * UDP discovery uses this state to pause and resume announcements.
 */
bool websocketServiceHasClient();


/*
 * Poll TCP input and drain at most one queued network message.
 */
void websocketServicePoll();
