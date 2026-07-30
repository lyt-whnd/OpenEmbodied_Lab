#ifndef ROBOT_TIME_H
#define ROBOT_TIME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


/*
 * Stage 2 timestamps are unsigned milliseconds since MCU boot.
 * Arithmetic is modulo 2^32; intervals must remain below 2^31 ms.
 */
#define ROBOT_TIME_UNIT_US            1000U
#define ROBOT_TIME_MAX_INTERVAL_MS    0x7FFFFFFFU


uint32_t RobotTime_ElapsedMs(
    uint32_t earlier_ms,
    uint32_t later_ms
);

bool RobotTime_Reached(
    uint32_t now_ms,
    uint32_t deadline_ms
);


#ifdef __cplusplus
}
#endif

#endif
