#pragma once

#include "esp_http_server.h"


/*
 * Wire the WebSocket transport, bounded network TX queue, V1 router, local
 * services, and STM32 link, then register /ws on the existing HTTP server.
 *
 * This coordinator does not parse WebSocket frames or Protocol V1 itself.
 */
bool websocketServiceRegister(
    httpd_handle_t server
);


/*
 * Return whether the transport still owns a live Linux WebSocket client.
 * UDP discovery uses this state to pause and resume announcements.
 */
bool websocketServiceHasClient();


/*
 * Drain at most one queue-owned network message from the Arduino loop.
 */
void websocketServicePoll();
