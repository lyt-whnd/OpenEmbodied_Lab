#pragma once

#include <stddef.h>
#include <stdint.h>


namespace ServiceRegistry
{

enum class QosClass : uint8_t
{
    BEST_EFFORT = 0,
    RELIABLE = 1,
    BULK = 2
};

enum class Priority : uint8_t
{
    BULK = 0,
    NORMAL = 1,
    HIGH = 2,
    EMERGENCY = 3
};

enum class OverflowPolicy : uint8_t
{
    DROP_OLD = 0,
    REJECT_NEW = 1,
    PAUSE = 2
};

struct MessagePolicy
{
    uint8_t service;
    uint8_t opcode;
    QosClass qos;
    Priority priority;
    uint16_t deadlineMs;
    OverflowPolicy overflow;
};

const MessagePolicy *lookup(
    uint8_t service,
    uint8_t opcode
);

size_t count();

}
