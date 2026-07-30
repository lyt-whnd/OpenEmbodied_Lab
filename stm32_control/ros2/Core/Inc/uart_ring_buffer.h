#ifndef UART_RING_BUFFER_H
#define UART_RING_BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_limits.h"


typedef struct
{
    volatile uint8_t data[
        ROBOT_UART_RX_RING_CAPACITY
    ];

    /*
     * Single producer (UART ISR) owns head; single consumer (main) owns tail.
     * Aligned 16-bit loads/stores are atomic on Cortex-M3.
     */
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint32_t overflow_count;
} UartRingBuffer;


void UartRing_Init(UartRingBuffer *ring);


bool UartRing_PushFromIsr(
    UartRingBuffer *ring,
    uint8_t byte
);


bool UartRing_Pop(
    UartRingBuffer *ring,
    uint8_t *byte
);


uint16_t UartRing_Count(
    const UartRingBuffer *ring
);


#ifdef __cplusplus
}
#endif

#endif
