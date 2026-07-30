#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "robot_protocol.h"


#define ROBOT_MOTION_YAW_CENTER_X10       900
#define ROBOT_MOTION_PITCH_CENTER_X10     900
#define ROBOT_MOTION_YAW_MIN_X10          200
#define ROBOT_MOTION_YAW_MAX_X10          1600
#define ROBOT_MOTION_PITCH_MIN_X10        300
#define ROBOT_MOTION_PITCH_MAX_X10        1500
#define ROBOT_MOTION_MAX_HEAD_RATE_X10    900
#define ROBOT_MOTION_MAX_VALID_MS         1000U


typedef void (*RobotMotionServoHandler)(
    uint8_t yaw_angle,
    uint8_t pitch_angle,
    void *user_context
);


/*
 * Motion owns only deterministic motion state and hardware application.
 * Protocol framing, message dispatch, and response transmission live outside.
 */
typedef struct
{
    RobotMotionServoHandler servo_handler;
    void *user_context;

    int32_t yaw_x10;
    int32_t pitch_x10;
    int32_t yaw_remainder;
    int32_t pitch_remainder;
    int16_t yaw_rate_x10;
    int16_t pitch_rate_x10;

    uint32_t current_time_ms;
    uint32_t last_update_ms;
    uint32_t command_deadline_ms;

    uint16_t control_epoch;
    uint16_t last_move_sequence;
    uint8_t last_applied_yaw;
    uint8_t last_applied_pitch;

    bool has_control_epoch;
    bool has_last_move_sequence;
    bool motion_active;
    bool estop_latched;
    bool motion_timed_out;
} RobotMotionController;


void RobotMotion_Init(
    RobotMotionController *controller,
    RobotMotionServoHandler servo_handler,
    void *user_context,
    uint32_t now_ms
);


/*
 * Handle one already decoded MOTION message and return its application result.
 * This function never frames, transmits, or dispatches a protocol message.
 */
RobotStatusCode RobotMotion_HandleMessage(
    RobotMotionController *controller,
    const RobotProtocolMessage *message
);


void RobotMotion_Process(
    RobotMotionController *controller,
    uint32_t now_ms
);


uint8_t RobotMotion_GetYawAngle(
    const RobotMotionController *controller
);


uint8_t RobotMotion_GetPitchAngle(
    const RobotMotionController *controller
);


#ifdef __cplusplus
}
#endif

#endif
