#include "robot_time.h"


uint32_t RobotTime_ElapsedMs(
    uint32_t earlier_ms,
    uint32_t later_ms
)
{
    return later_ms - earlier_ms;
}


bool RobotTime_Reached(
    uint32_t now_ms,
    uint32_t deadline_ms
)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}
