#ifndef ROBOT_SENSOR_H
#define ROBOT_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_limits.h"
#include "robot_protocol.h"
#include "robot_service_registry.h"
#include "sensor_schema.h"


typedef struct
{
    uint32_t timestamp_ms;
    uint16_t sample_sequence;
    uint16_t length;
    uint8_t data[ROBOT_SENSOR_MAX_SAMPLE_BYTES];
} RobotSensorSample;

typedef bool (*RobotSensorInitHandler)(void *context);
typedef bool (*RobotSensorStartHandler)(void *context);
typedef void (*RobotSensorStopHandler)(void *context);
typedef RobotStatusCode (*RobotSensorConfigureHandler)(
    uint16_t key,
    int32_t value,
    void *context
);
typedef bool (*RobotSensorPollHandler)(
    uint32_t now_ms,
    RobotSensorSample *sample,
    void *context
);
typedef void (*RobotSensorGetInfoHandler)(
    uint8_t *output,
    uint16_t capacity,
    uint16_t *length,
    void *context
);

typedef struct
{
    RobotSensorInitHandler init;
    RobotSensorStartHandler start;
    RobotSensorStopHandler stop;
    RobotSensorConfigureHandler configure;
    RobotSensorPollHandler poll_or_read;
    RobotSensorGetInfoHandler get_info;
} RobotSensorDriver;

typedef struct
{
    uint16_t sensor_type;
    uint8_t instance_id;
    uint8_t schema_version;
    RobotQosClass default_qos;
    RobotSensorBufferPolicy buffer_policy;
    uint16_t sample_size;
    uint16_t sample_period_ms;
    uint16_t max_latency_ms;
    RobotSensorDriver driver;
    void *driver_context;
} RobotSensorDescriptor;

typedef struct
{
    const RobotSensorDescriptor *descriptors;
    uint8_t count;
    bool initialized[ROBOT_SENSOR_MAX_REGISTERED];
    bool running[ROBOT_SENSOR_MAX_REGISTERED];
} RobotSensorRegistry;


bool RobotSensorRegistry_Init(
    RobotSensorRegistry *registry,
    const RobotSensorDescriptor *descriptors,
    uint8_t count
);

const RobotSensorDescriptor *RobotSensorRegistry_Find(
    const RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id,
    uint8_t *index
);

RobotStatusCode RobotSensorRegistry_Start(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id
);

RobotStatusCode RobotSensorRegistry_Stop(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id
);

RobotStatusCode RobotSensorRegistry_Configure(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id,
    uint16_t key,
    int32_t value
);


#ifdef __cplusplus
}
#endif

#endif
