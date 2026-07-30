#ifndef ROBOT_TX_SCHEDULER_H
#define ROBOT_TX_SCHEDULER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_limits.h"
#include "robot_service_registry.h"


typedef enum
{
    ROBOT_TX_RELIABLE = 0,
    ROBOT_TX_BEST_EFFORT_LATEST,
    ROBOT_TX_BEST_EFFORT_SAMPLE,
    ROBOT_TX_BULK
} RobotTxStorageClass;

typedef struct
{
    uint32_t enqueued;
    uint32_t dequeued;
    uint32_t dropped_full;
    uint32_t replaced_latest;
    uint32_t dropped_sample;
    uint32_t expired;
    uint32_t rejected_oversize;
} RobotTxSchedulerStats;

typedef struct
{
    uint8_t bytes[ROBOT_TX_RELIABLE_MAX_BYTES];
    uint16_t length;
    uint32_t deadline_ms;
    uint32_t order;
    uint32_t key;
    RobotPriority priority;
    bool has_deadline;
    bool occupied;
} RobotTxReliableSlot;

typedef struct
{
    uint8_t bytes[ROBOT_TX_LATEST_MAX_BYTES];
    uint16_t length;
    uint32_t deadline_ms;
    uint32_t order;
    uint32_t key;
    RobotPriority priority;
    bool has_deadline;
    bool occupied;
} RobotTxLatestSlot;

typedef struct
{
    uint8_t bytes[ROBOT_TX_SAMPLE_MAX_BYTES];
    uint16_t length;
    uint32_t deadline_ms;
    uint32_t order;
    uint32_t key;
    RobotPriority priority;
    bool has_deadline;
    bool occupied;
} RobotTxSampleSlot;

typedef struct
{
    uint8_t bytes[ROBOT_TX_BULK_MAX_BYTES];
    uint16_t length;
    uint32_t deadline_ms;
    uint32_t order;
    uint32_t key;
    RobotPriority priority;
    bool has_deadline;
    bool occupied;
} RobotTxBulkSlot;

typedef struct
{
    RobotTxReliableSlot reliable[ROBOT_TX_RELIABLE_CAPACITY];
    RobotTxLatestSlot latest[ROBOT_TX_LATEST_CAPACITY];
    RobotTxSampleSlot samples[ROBOT_TX_SAMPLE_CAPACITY];
    RobotTxBulkSlot bulk[ROBOT_TX_BULK_CAPACITY];
    uint32_t next_order;
    RobotTxSchedulerStats stats;
} RobotTxScheduler;


void RobotTxScheduler_Init(RobotTxScheduler *scheduler);

bool RobotTxScheduler_Enqueue(
    RobotTxScheduler *scheduler,
    RobotTxStorageClass storage,
    const RobotMessagePolicy *policy,
    uint32_t key,
    const uint8_t *data,
    uint16_t length,
    uint32_t now_ms
);

bool RobotTxScheduler_TakeNext(
    RobotTxScheduler *scheduler,
    uint32_t now_ms,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
);

uint16_t RobotTxScheduler_Pending(
    const RobotTxScheduler *scheduler
);


#ifdef __cplusplus
}
#endif

#endif
