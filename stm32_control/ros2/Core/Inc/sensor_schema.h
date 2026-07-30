#ifndef SENSOR_SCHEMA_H
#define SENSOR_SCHEMA_H

#include <stdint.h>


#define ROBOT_SENSOR_SCHEMA_VERSION 1U
#define ROBOT_SENSOR_TARGET_SIZE    3U

typedef enum
{
    ROBOT_SENSOR_TYPE_VIRTUAL_COUNTER = 0x0001,
    ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL   = 0x0002
} RobotSensorType;

typedef enum
{
    ROBOT_SENSOR_BUFFER_LATEST = 0,
    ROBOT_SENSOR_BUFFER_SAMPLE_RING = 1,
    ROBOT_SENSOR_BUFFER_EVENT = 2
} RobotSensorBufferPolicy;

typedef struct
{
    uint16_t sensor_type;
    uint8_t instance_id;
} RobotSensorTarget;


#endif
