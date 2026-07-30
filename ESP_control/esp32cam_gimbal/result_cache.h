#pragma once

#include <stddef.h>
#include <stdint.h>


namespace Reliable
{

static constexpr size_t RESULT_CACHE_CAPACITY = 8U;
static constexpr uint32_t RESULT_CACHE_TTL_MS = 30000U;

enum class ResultStage : uint8_t
{
    RECEIVED = 1,
    APPLIED = 2,
    FAILED = 3
};

struct CachedResult
{
    ResultStage stage;
    uint16_t status;
};

enum class CacheLookup : uint8_t
{
    MISS,
    FOUND,
    CONFLICT
};

class ResultCache
{
public:
    ResultCache();

    void reset();

    CacheLookup find(
        uint8_t src,
        uint8_t service,
        uint8_t opcode,
        uint16_t epoch,
        uint32_t requestId,
        uint32_t nowMs,
        CachedResult &result
    );

    void store(
        uint8_t src,
        uint8_t service,
        uint8_t opcode,
        uint16_t epoch,
        uint32_t requestId,
        ResultStage stage,
        uint16_t status,
        uint32_t nowMs
    );

private:
    struct Entry
    {
        bool valid;
        uint8_t src;
        uint8_t service;
        uint8_t opcode;
        uint16_t epoch;
        uint32_t requestId;
        ResultStage stage;
        uint16_t status;
        uint32_t touchedMs;
    };

    Entry entries_[RESULT_CACHE_CAPACITY];
};

}
