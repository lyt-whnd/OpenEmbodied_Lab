#include "result_cache.h"

#include <string.h>


namespace Reliable
{

namespace
{

bool expired(uint32_t nowMs, uint32_t touchedMs)
{
    return static_cast<uint32_t>(
        nowMs - touchedMs
    ) >= RESULT_CACHE_TTL_MS;
}

}


ResultCache::ResultCache()
{
    reset();
}


void ResultCache::reset()
{
    memset(entries_, 0, sizeof(entries_));
}


CacheLookup ResultCache::find(
    uint8_t src,
    uint8_t service,
    uint8_t opcode,
    uint16_t epoch,
    uint32_t requestId,
    uint32_t nowMs,
    CachedResult &result
)
{
    result = {};

    for (Entry &entry : entries_)
    {
        if (entry.valid && expired(nowMs, entry.touchedMs))
        {
            entry.valid = false;
        }

        if (
            !entry.valid ||
            entry.src != src ||
            entry.epoch != epoch ||
            entry.requestId != requestId
        )
        {
            continue;
        }

        if (
            entry.service != service ||
            entry.opcode != opcode
        )
        {
            return CacheLookup::CONFLICT;
        }

        entry.touchedMs = nowMs;
        result.stage = entry.stage;
        result.status = entry.status;
        return CacheLookup::FOUND;
    }

    return CacheLookup::MISS;
}


void ResultCache::store(
    uint8_t src,
    uint8_t service,
    uint8_t opcode,
    uint16_t epoch,
    uint32_t requestId,
    ResultStage stage,
    uint16_t status,
    uint32_t nowMs
)
{
    Entry *target = nullptr;

    for (Entry &entry : entries_)
    {
        if (!entry.valid || expired(nowMs, entry.touchedMs))
        {
            target = &entry;
            break;
        }

        if (
            target == nullptr ||
            static_cast<int32_t>(
                entry.touchedMs - target->touchedMs
            ) < 0
        )
        {
            target = &entry;
        }
    }

    if (target == nullptr)
    {
        target = &entries_[0];
    }

    target->valid = true;
    target->src = src;
    target->service = service;
    target->opcode = opcode;
    target->epoch = epoch;
    target->requestId = requestId;
    target->stage = stage;
    target->status = status;
    target->touchedMs = nowMs;
}

}
