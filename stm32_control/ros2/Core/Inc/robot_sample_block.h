#ifndef ROBOT_SAMPLE_BLOCK_H
#define ROBOT_SAMPLE_BLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_sensor.h"
#include "robot_time.h"


#define ROBOT_SAMPLE_BLOCK_SCHEMA_VERSION 1U
#define ROBOT_SAMPLE_BLOCK_HEADER_SIZE    18U

typedef struct
{
    const RobotSensorDescriptor *descriptor;
    uint8_t registry_index;
    uint8_t samples[
        ROBOT_SAMPLE_RING_CAPACITY
    ][ROBOT_SAMPLE_MAX_BYTES];
    uint32_t timestamps_ms[ROBOT_SAMPLE_RING_CAPACITY];
    uint8_t head;
    uint8_t count;
    uint16_t dropped_since_block;
} RobotSampleRing;

typedef struct
{
    RobotSampleRing streams[ROBOT_SAMPLE_STREAM_CAPACITY];
    uint8_t stream_count;
    uint32_t pushed;
    uint32_t blocks_built;
    uint32_t samples_built;
    uint32_t dropped;
    uint32_t rejected;
} RobotSampleManager;


bool RobotSampleManager_Init(
    RobotSampleManager *manager,
    const RobotSensorRegistry *registry
);

uint8_t RobotSampleManager_Poll(
    RobotSampleManager *manager,
    RobotSensorRegistry *registry,
    uint32_t now_ms
);

bool RobotSampleManager_Push(
    RobotSampleManager *manager,
    uint8_t stream_index,
    const RobotSensorSample *sample
);

bool RobotSampleManager_BuildNext(
    RobotSampleManager *manager,
    uint32_t now_ms,
    bool force,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
);


#ifdef __cplusplus
}
#endif

#endif
