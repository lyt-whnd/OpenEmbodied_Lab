#pragma once

#include <stddef.h>
#include <stdint.h>


using TcpTransportReceiveCallback = void (*)(
    const uint8_t *data,
    size_t length
);


/*
 * Start the single-client V1 TCP server configured in app_config.h.
 *
 * Each packet is carried as:
 *     uint16 little-endian packet length + complete opaque V1 packet
 */
bool tcpTransportBegin();


void tcpTransportSetReceiveCallback(
    TcpTransportReceiveCallback callback
);


bool tcpTransportSend(
    const uint8_t *data,
    size_t length
);


bool tcpTransportHasClient();


/*
 * Accept a client and consume all currently available stream bytes.
 * Call from the Arduino loop; this function never waits for a full packet.
 */
void tcpTransportPoll();
