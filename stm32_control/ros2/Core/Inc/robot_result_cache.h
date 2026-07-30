#ifndef ROBOT_RESULT_CACHE_H
#define ROBOT_RESULT_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "robot_limits.h"


typedef enum
{
    ROBOT_RESULT_STAGE_RECEIVED = 1,
    ROBOT_RESULT_STAGE_APPLIED = 2,
    ROBOT_RESULT_STAGE_FAILED = 3
} RobotResultStage;

typedef struct
{
    uint8_t valid;
    uint8_t src;
    uint8_t service;
    uint8_t opcode;
    uint16_t epoch;
    uint32_t request_id;
    RobotResultStage stage;
    uint16_t status;
    uint32_t touched_ms;
} RobotResultCacheEntry;

typedef struct
{
    RobotResultCacheEntry entries[
        ROBOT_RESULT_CACHE_CAPACITY
    ];
} RobotResultCache;

typedef enum
{
    ROBOT_RESULT_CACHE_MISS,
    ROBOT_RESULT_CACHE_FOUND,
    ROBOT_RESULT_CACHE_CONFLICT
} RobotResultCacheLookup;

void RobotResultCache_Init(RobotResultCache *cache);

RobotResultCacheLookup RobotResultCache_Find(
    RobotResultCache *cache,
    uint8_t src,
    uint8_t service,
    uint8_t opcode,
    uint16_t epoch,
    uint32_t request_id,
    uint32_t now_ms,
    RobotResultCacheEntry *result
);

void RobotResultCache_Store(
    RobotResultCache *cache,
    uint8_t src,
    uint8_t service,
    uint8_t opcode,
    uint16_t epoch,
    uint32_t request_id,
    RobotResultStage stage,
    uint16_t status,
    uint32_t now_ms
);


#ifdef __cplusplus
}
#endif

#endif
