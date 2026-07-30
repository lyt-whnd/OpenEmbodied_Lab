#ifndef ROBOT_DISPATCHER_H
#define ROBOT_DISPATCHER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_motion.h"
#include "robot_protocol.h"


typedef struct
{
    RobotProtocolContext protocol;
    RobotMotionController *motion;

    RobotProtocolTransmitHandler transmit_handler;
    void *transport_context;
    uint32_t current_time_ms;

    uint32_t dispatched_message_count;
    uint32_t unsupported_service_count;
    uint32_t response_error_count;
} RobotDispatcher;


void RobotDispatcher_Init(
    RobotDispatcher *dispatcher,
    RobotMotionController *motion,
    RobotProtocolTransmitHandler transmit_handler,
    void *transport_context,
    uint32_t now_ms
);


void RobotDispatcher_SetTime(
    RobotDispatcher *dispatcher,
    uint32_t now_ms
);


RobotStatusCode RobotDispatcher_OnMessage(
    RobotDispatcher *dispatcher,
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length
);


RobotProtocolContext *RobotDispatcher_GetProtocol(
    RobotDispatcher *dispatcher
);


#ifdef __cplusplus
}
#endif

#endif
