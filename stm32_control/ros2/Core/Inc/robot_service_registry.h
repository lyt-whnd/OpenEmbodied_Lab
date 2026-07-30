#ifndef ROBOT_SERVICE_REGISTRY_H
#define ROBOT_SERVICE_REGISTRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


typedef enum
{
    ROBOT_QOS_BEST_EFFORT = 0,
    ROBOT_QOS_RELIABLE = 1,
    ROBOT_QOS_BULK = 2
} RobotQosClass;

typedef enum
{
    ROBOT_PRIORITY_BULK = 0,
    ROBOT_PRIORITY_NORMAL = 1,
    ROBOT_PRIORITY_HIGH = 2,
    ROBOT_PRIORITY_EMERGENCY = 3
} RobotPriority;

typedef enum
{
    ROBOT_OVERFLOW_DROP_OLD = 0,
    ROBOT_OVERFLOW_REJECT_NEW = 1,
    ROBOT_OVERFLOW_PAUSE = 2
} RobotOverflowPolicy;

typedef struct
{
    uint8_t service;
    uint8_t opcode;
    RobotQosClass qos;
    RobotPriority priority;
    uint16_t deadline_ms;
    RobotOverflowPolicy overflow;
} RobotMessagePolicy;

const RobotMessagePolicy *RobotServiceRegistry_Lookup(
    uint8_t service,
    uint8_t opcode
);

bool RobotServiceRegistry_HasService(uint8_t service);

size_t RobotServiceRegistry_Count(void);


#ifdef __cplusplus
}
#endif

#endif
