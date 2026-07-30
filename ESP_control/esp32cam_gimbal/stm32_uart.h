#pragma once

#include <stddef.h>
#include <stdint.h>


using Stm32TransportReceiveCallback = void (*)(
    const uint8_t *data,
    size_t length
);


/*
 * Initialize the ESP32-CAM UART1 link to STM32.
 */
bool stm32TransportInit();


/*
 * Register a callback for one CRC-checked application frame.
 *
 * data is borrowed and remains valid only during the callback. Protocol V1
 * validation and routing belong to the message router.
 */
void stm32TransportSetReceiveCallback(
    Stm32TransportReceiveCallback callback
);


/*
 * Add COBS + CRC16-CCITT-FALSE framing and send opaque application bytes.
 */
bool stm32TransportSend(
    const uint8_t *data,
    size_t length
);


/*
 * Non-blocking UART polling with 0x00 framing and overflow resynchronization.
 */
void stm32TransportPoll();
