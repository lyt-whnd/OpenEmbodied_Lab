#include "robot_result_cache.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>


static bool entry_expired(
    const RobotResultCacheEntry *entry,
    uint32_t now_ms
)
{
    return (
        entry->valid != 0U &&
        (uint32_t)(now_ms - entry->touched_ms) >=
            ROBOT_RESULT_CACHE_TTL_MS
    );
}


void RobotResultCache_Init(RobotResultCache *cache)
{
    if (cache != NULL)
    {
        memset(cache, 0, sizeof(*cache));
    }
}


RobotResultCacheLookup RobotResultCache_Find(
    RobotResultCache *cache,
    uint8_t src,
    uint8_t service,
    uint8_t opcode,
    uint16_t epoch,
    uint32_t request_id,
    uint32_t now_ms,
    RobotResultCacheEntry *result
)
{
    size_t index;

    if (cache == NULL)
    {
        return ROBOT_RESULT_CACHE_MISS;
    }

    for (
        index = 0U;
        index < ROBOT_RESULT_CACHE_CAPACITY;
        ++index
    )
    {
        RobotResultCacheEntry *entry =
            &cache->entries[index];

        if (entry_expired(entry, now_ms))
        {
            entry->valid = 0U;
        }

        if (
            entry->valid == 0U ||
            entry->src != src ||
            entry->epoch != epoch ||
            entry->request_id != request_id
        )
        {
            continue;
        }

        if (
            entry->service != service ||
            entry->opcode != opcode
        )
        {
            return ROBOT_RESULT_CACHE_CONFLICT;
        }

        entry->touched_ms = now_ms;

        if (result != NULL)
        {
            *result = *entry;
        }

        return ROBOT_RESULT_CACHE_FOUND;
    }

    return ROBOT_RESULT_CACHE_MISS;
}


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
)
{
    RobotResultCacheEntry *target = NULL;
    size_t index;

    if (cache == NULL)
    {
        return;
    }

    for (
        index = 0U;
        index < ROBOT_RESULT_CACHE_CAPACITY;
        ++index
    )
    {
        RobotResultCacheEntry *entry =
            &cache->entries[index];

        if (
            entry->valid == 0U ||
            entry_expired(entry, now_ms)
        )
        {
            target = entry;
            break;
        }

        if (
            target == NULL ||
            (int32_t)(
                entry->touched_ms -
                target->touched_ms
            ) < 0
        )
        {
            target = entry;
        }
    }

    if (target == NULL)
    {
        target = &cache->entries[0];
    }

    target->valid = 1U;
    target->src = src;
    target->service = service;
    target->opcode = opcode;
    target->epoch = epoch;
    target->request_id = request_id;
    target->stage = stage;
    target->status = status;
    target->touched_ms = now_ms;
}
