#include "robot_transport.h"

#include <stddef.h>
#include <string.h>

#include "robot_limits.h"


static bool arm_next_receive(
    RobotTransport *transport
)
{
    if (
        transport == NULL ||
        transport->arm_receive_handler == NULL ||
        !transport->arm_receive_handler(
            &transport->interrupt_byte,
            transport->user_context
        )
    )
    {
        if (transport != NULL)
        {
            transport->rx_armed = false;
            ++transport->rx_rearm_error_count;
        }

        return false;
    }

    transport->rx_armed = true;
    return true;
}


void RobotTransport_Init(
    RobotTransport *transport,
    RobotProtocolContext *protocol,
    RobotTransportArmReceiveHandler arm_receive_handler,
    RobotTransportWriteHandler write_handler,
    RobotTransportClockHandler clock_handler,
    void *user_context
)
{
    if (transport == NULL)
    {
        return;
    }

    transport->rx_armed = false;

    memset(transport, 0, sizeof(*transport));
    UartRing_Init(&transport->rx_ring);

    transport->protocol = protocol;
    transport->arm_receive_handler =
        arm_receive_handler;
    transport->write_handler = write_handler;
    transport->clock_handler = clock_handler;
    transport->user_context = user_context;
}


bool RobotTransport_StartRx(
    RobotTransport *transport
)
{
    return arm_next_receive(transport);
}


void RobotTransport_OnRxCompleteFromIsr(
    RobotTransport *transport
)
{
    if (transport == NULL)
    {
        return;
    }

    if (
        UartRing_PushFromIsr(
            &transport->rx_ring,
            transport->interrupt_byte
        )
    )
    {
        ++transport->rx_byte_count;
    }

    (void)arm_next_receive(transport);
}


void RobotTransport_OnErrorFromIsr(
    RobotTransport *transport
)
{
    if (transport == NULL)
    {
        return;
    }

    transport->rx_armed = false;
    ++transport->uart_error_count;
    (void)arm_next_receive(transport);
}


uint16_t RobotTransport_Poll(
    RobotTransport *transport
)
{
    uint16_t processed = 0U;
    uint8_t byte;

    if (
        transport == NULL ||
        transport->protocol == NULL
    )
    {
        return 0U;
    }

    /*
     * Recover if HAL could not re-arm inside a completion/error callback.
     * On Cortex-M3 an ISR completes before main resumes, so this cannot race
     * the callback's own re-arm attempt.
     */
    if (!transport->rx_armed)
    {
        (void)arm_next_receive(transport);
    }

    while (
        processed <
            ROBOT_TRANSPORT_MAX_POLL_BYTES &&
        UartRing_Pop(
            &transport->rx_ring,
            &byte
        )
    )
    {
        RobotProtocol_InputByte(
            transport->protocol,
            byte
        );
        ++processed;
    }

    return processed;
}


bool RobotTransport_Transmit(
    const uint8_t *data,
    uint16_t length,
    void *user_context
)
{
    RobotTransport *transport =
        (RobotTransport *)user_context;
    uint32_t start_ms = 0U;
    uint32_t elapsed_ms = 0U;
    bool result;

    if (
        transport == NULL ||
        transport->write_handler == NULL ||
        data == NULL ||
        length == 0U
    )
    {
        if (transport != NULL)
        {
            ++transport->tx_error_count;
        }

        return false;
    }

    if (transport->clock_handler != NULL)
    {
        start_ms = transport->clock_handler(
            transport->user_context
        );
    }

    result = transport->write_handler(
        data,
        length,
        ROBOT_UART_TX_TIMEOUT_MS,
        transport->user_context
    );

    if (transport->clock_handler != NULL)
    {
        elapsed_ms =
            transport->clock_handler(
                transport->user_context
            ) -
            start_ms;

        if (
            elapsed_ms >
            transport->tx_max_duration_ms
        )
        {
            transport->tx_max_duration_ms =
                elapsed_ms;
        }
    }

    if (result)
    {
        ++transport->tx_success_count;
    }
    else
    {
        ++transport->tx_error_count;
    }

    return result;
}
