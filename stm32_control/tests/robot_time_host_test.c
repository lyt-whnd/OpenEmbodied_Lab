#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "robot_time.h"


int main(void)
{
    assert(RobotTime_ElapsedMs(100U, 125U) == 25U);
    assert(
        RobotTime_ElapsedMs(
            0xFFFFFFF0U,
            0x00000010U
        ) == 32U
    );
    assert(RobotTime_Reached(100U, 100U));
    assert(RobotTime_Reached(101U, 100U));
    assert(
        RobotTime_Reached(
            0x00000010U,
            0xFFFFFFF0U
        )
    );
    assert(!RobotTime_Reached(99U, 100U));
    puts("STM32 monotonic time host tests passed");
    return 0;
}
