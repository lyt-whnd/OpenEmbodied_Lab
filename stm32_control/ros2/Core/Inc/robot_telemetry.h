#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_sensor.h"
#include "robot_time.h"


#define ROBOT_SENSOR_RECORD_HEADER_SIZE 12U
#define ROBOT_TELEMETRY_BATCH_HEADER_SIZE 2U
#define ROBOT_TELEMETRY_BATCH_SCHEMA_VERSION 1U

typedef struct
{
    uint16_t sensor_type;
    uint8_t instance_id;
    uint8_t schema_version;
    uint32_t timestamp_ms;
    uint16_t sample_sequence;
    uint16_t data_length;
    uint16_t max_latency_ms;
    uint32_t published_at_ms;
    uint8_t data[ROBOT_SENSOR_MAX_SAMPLE_BYTES];
    bool pending;
} RobotTelemetryLatestSlot;

typedef struct
{
    RobotTelemetryLatestSlot
        latest[ROBOT_TELEMETRY_LATEST_CAPACITY];
    uint32_t last_flush_ms;
    uint32_t published;
    uint32_t replaced;
    uint32_t rejected_full;
    uint32_t rejected_oversize;
    uint32_t batches_built;
    uint32_t records_built;
} RobotTelemetryAggregator;


void RobotTelemetry_Init(
    RobotTelemetryAggregator *aggregator,
    uint32_t now_ms
);

bool RobotTelemetry_PublishLatest(
    RobotTelemetryAggregator *aggregator,
    const RobotSensorDescriptor *descriptor,
    const RobotSensorSample *sample,
    uint32_t now_ms
);

uint8_t RobotTelemetry_PollRegistry(
    RobotTelemetryAggregator *aggregator,
    RobotSensorRegistry *registry,
    uint32_t now_ms
);

bool RobotTelemetry_ShouldFlush(
    const RobotTelemetryAggregator *aggregator,
    uint32_t now_ms
);

bool RobotTelemetry_BuildBatch(
    RobotTelemetryAggregator *aggregator,
    uint32_t now_ms,
    bool force,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
);

bool RobotTelemetry_RequiresImmediateSend(
    RobotSensorBufferPolicy buffer_policy,
    RobotPriority priority
);


#ifdef __cplusplus
}
#endif

#endif
