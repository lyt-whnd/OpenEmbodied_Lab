#ifndef ROBOT_TRANSPORT_H
#define ROBOT_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_protocol.h"
#include "uart_ring_buffer.h"


typedef bool (*RobotTransportArmReceiveHandler)(
    uint8_t *destination,
    void *user_context
);


typedef bool (*RobotTransportWriteHandler)(
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout_ms,
    void *user_context
);


typedef uint32_t (*RobotTransportClockHandler)(
    void *user_context
);


typedef struct
{
    UartRingBuffer rx_ring;
    RobotProtocolContext *protocol;

    RobotTransportArmReceiveHandler arm_receive_handler;
    RobotTransportWriteHandler write_handler;
    RobotTransportClockHandler clock_handler;
    void *user_context;

    uint8_t interrupt_byte;
    volatile bool rx_armed;

    volatile uint32_t rx_byte_count;
    volatile uint32_t rx_rearm_error_count;
    volatile uint32_t uart_error_count;

    uint32_t tx_success_count;
    uint32_t tx_error_count;
    uint32_t tx_max_duration_ms;
} RobotTransport;


void RobotTransport_Init(
    RobotTransport *transport,
    RobotProtocolContext *protocol,
    RobotTransportArmReceiveHandler arm_receive_handler,
    RobotTransportWriteHandler write_handler,
    RobotTransportClockHandler clock_handler,
    void *user_context
);


bool RobotTransport_StartRx(
    RobotTransport *transport
);


void RobotTransport_OnRxCompleteFromIsr(
    RobotTransport *transport
);


void RobotTransport_OnErrorFromIsr(
    RobotTransport *transport
);


uint16_t RobotTransport_Poll(
    RobotTransport *transport
);


/*
 * Protocol TX callback. Stage 1 deliberately keeps bounded blocking UART TX
 * while recording success, failure, and maximum elapsed time.
 */
bool RobotTransport_Transmit(
    const uint8_t *data,
    uint16_t length,
    void *user_context
);


#ifdef __cplusplus
}
#endif

#endif
