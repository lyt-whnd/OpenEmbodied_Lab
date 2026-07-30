#include "uart_ring_buffer.h"

#include <stddef.h>
#include <string.h>


_Static_assert(
    (
        ROBOT_UART_RX_RING_CAPACITY &
        (ROBOT_UART_RX_RING_CAPACITY - 1U)
    ) == 0U,
    "UART RX ring capacity must be a power of two"
);

_Static_assert(
    ROBOT_UART_RX_RING_CAPACITY <= UINT16_MAX,
    "UART RX ring exceeds 16-bit index distance"
);


void UartRing_Init(UartRingBuffer *ring)
{
    if (ring == NULL)
    {
        return;
    }

    memset(ring, 0, sizeof(*ring));
}


bool UartRing_PushFromIsr(
    UartRingBuffer *ring,
    uint8_t byte
)
{
    uint16_t head;
    uint16_t tail;

    if (ring == NULL)
    {
        return false;
    }

    head = ring->head;
    tail = ring->tail;

    if (
        (uint16_t)(head - tail) >=
        ROBOT_UART_RX_RING_CAPACITY
    )
    {
        ++ring->overflow_count;
        return false;
    }

    ring->data[
        head &
        (ROBOT_UART_RX_RING_CAPACITY - 1U)
    ] = byte;
    ring->head = (uint16_t)(head + 1U);

    return true;
}


bool UartRing_Pop(
    UartRingBuffer *ring,
    uint8_t *byte
)
{
    uint16_t tail;

    if (ring == NULL || byte == NULL)
    {
        return false;
    }

    tail = ring->tail;

    if (tail == ring->head)
    {
        return false;
    }

    *byte = ring->data[
        tail &
        (ROBOT_UART_RX_RING_CAPACITY - 1U)
    ];
    ring->tail = (uint16_t)(tail + 1U);

    return true;
}


uint16_t UartRing_Count(
    const UartRingBuffer *ring
)
{
    if (ring == NULL)
    {
        return 0U;
    }

    return (uint16_t)(
        ring->head - ring->tail
    );
}
