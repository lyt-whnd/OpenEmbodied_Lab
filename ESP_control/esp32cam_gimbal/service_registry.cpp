#include "service_registry.h"


namespace ServiceRegistry
{

namespace
{

const MessagePolicy policies[] =
{
#include "service_registry_generated.inc"
};

}


const MessagePolicy *lookup(
    uint8_t service,
    uint8_t opcode
)
{
    for (const MessagePolicy &policy : policies)
    {
        if (
            policy.service == service &&
            policy.opcode == opcode
        )
        {
            return &policy;
        }
    }

    return nullptr;
}


size_t count()
{
    return sizeof(policies) / sizeof(policies[0]);
}

}
