#include "robot_service_registry.h"

#include <stddef.h>


static const RobotMessagePolicy policies[] =
{
#include "robot_service_registry_generated.inc"
};


const RobotMessagePolicy *RobotServiceRegistry_Lookup(
    uint8_t service,
    uint8_t opcode
)
{
    size_t index;

    for (
        index = 0U;
        index < RobotServiceRegistry_Count();
        ++index
    )
    {
        if (
            policies[index].service == service &&
            policies[index].opcode == opcode
        )
        {
            return &policies[index];
        }
    }

    return NULL;
}


bool RobotServiceRegistry_HasService(uint8_t service)
{
    size_t index;

    for (
        index = 0U;
        index < RobotServiceRegistry_Count();
        ++index
    )
    {
        if (policies[index].service == service)
        {
            return true;
        }
    }

    return false;
}


size_t RobotServiceRegistry_Count(void)
{
    return sizeof(policies) / sizeof(policies[0]);
}
