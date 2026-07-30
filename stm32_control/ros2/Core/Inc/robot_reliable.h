#ifndef ROBOT_RELIABLE_H
#define ROBOT_RELIABLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_protocol.h"
#include "robot_result_cache.h"


#define ROBOT_RELIABLE_SCHEMA_VERSION    1U
#define ROBOT_RELIABLE_REQUEST_SIZE      7U
#define ROBOT_RELIABLE_RESULT_SIZE       10U


typedef struct
{
    uint16_t epoch;
    uint32_t request_id;
    const uint8_t *payload;
    uint16_t payload_length;
} RobotReliableRequest;


bool RobotReliable_DecodeRequest(
    const RobotProtocolMessage *message,
    RobotReliableRequest *request
);

bool RobotReliable_SendResult(
    RobotProtocolContext *protocol,
    const RobotProtocolMessage *request_message,
    const RobotReliableRequest *request,
    RobotResultStage stage,
    RobotStatusCode status
);


#ifdef __cplusplus
}
#endif

#endif
