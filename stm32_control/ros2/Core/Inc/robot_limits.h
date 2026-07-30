#ifndef ROBOT_LIMITS_H
#define ROBOT_LIMITS_H


/*
 * All communication storage is compile-time bounded for STM32F103C8T6.
 * The RX ring capacity must remain a power of two.
 */
#define ROBOT_UART_RX_RING_CAPACITY       256U
#define ROBOT_TRANSPORT_MAX_POLL_BYTES    256U
#define ROBOT_UART_TX_TIMEOUT_MS          100U


#endif
