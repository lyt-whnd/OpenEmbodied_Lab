#include "robot_motion.h"

#include <stddef.h>
#include <string.h>


static int32_t clamp_i32(
    int32_t value,
    int32_t minimum,
    int32_t maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


static bool time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}


static bool sequence_is_newer(uint16_t sequence, uint16_t last)
{
    return (int16_t)(sequence - last) > 0;
}


static uint8_t angle_x10_to_degrees(int32_t angle_x10)
{
    return (uint8_t)((angle_x10 + 5) / 10);
}


static void apply_servo_angles(
    RobotMotionController *controller
)
{
    uint8_t yaw;
    uint8_t pitch;

    controller->yaw_x10 = clamp_i32(
        controller->yaw_x10,
        ROBOT_MOTION_YAW_MIN_X10,
        ROBOT_MOTION_YAW_MAX_X10
    );
    controller->pitch_x10 = clamp_i32(
        controller->pitch_x10,
        ROBOT_MOTION_PITCH_MIN_X10,
        ROBOT_MOTION_PITCH_MAX_X10
    );

    yaw = angle_x10_to_degrees(controller->yaw_x10);
    pitch = angle_x10_to_degrees(controller->pitch_x10);

    if (
        controller->servo_handler != NULL &&
        (
            yaw != controller->last_applied_yaw ||
            pitch != controller->last_applied_pitch
        )
    )
    {
        controller->servo_handler(
            yaw,
            pitch,
            controller->user_context
        );
        controller->last_applied_yaw = yaw;
        controller->last_applied_pitch = pitch;
    }
}


static void stop_motion(
    RobotMotionController *controller
)
{
    controller->motion_active = false;
    controller->yaw_rate_x10 = 0;
    controller->pitch_rate_x10 = 0;
    controller->yaw_remainder = 0;
    controller->pitch_remainder = 0;
}


static RobotStatusCode validate_empty_payload(
    const RobotProtocolMessage *message
)
{
    if (message->payload_length != 0U)
    {
        return ROBOT_STATUS_BAD_LENGTH;
    }

    return ROBOT_STATUS_OK;
}


static RobotStatusCode handle_move(
    RobotMotionController *controller,
    const RobotProtocolMessage *message
)
{
    RobotMotionMovePayload move;

    if (!RobotProtocol_DecodeMotionMove(message, &move))
    {
        return ROBOT_STATUS_BAD_LENGTH;
    }

    if (controller->estop_latched)
    {
        return ROBOT_STATUS_ESTOP_ACTIVE;
    }

    if (
        move.valid_ms == 0U ||
        move.valid_ms > ROBOT_MOTION_MAX_VALID_MS ||
        move.head_yaw_rate_x10 <
            -ROBOT_MOTION_MAX_HEAD_RATE_X10 ||
        move.head_yaw_rate_x10 >
            ROBOT_MOTION_MAX_HEAD_RATE_X10 ||
        move.head_pitch_rate_x10 <
            -ROBOT_MOTION_MAX_HEAD_RATE_X10 ||
        move.head_pitch_rate_x10 >
            ROBOT_MOTION_MAX_HEAD_RATE_X10
    )
    {
        return ROBOT_STATUS_OUT_OF_RANGE;
    }

    if (
        move.linear_mm_s != 0 ||
        move.angular_mrad_s != 0
    )
    {
        return ROBOT_STATUS_NOT_IMPLEMENTED;
    }

    if (
        !controller->has_control_epoch ||
        move.control_epoch != controller->control_epoch
    )
    {
        controller->control_epoch = move.control_epoch;
        controller->has_control_epoch = true;
        controller->has_last_move_sequence = false;
    }

    if (
        controller->has_last_move_sequence &&
        !sequence_is_newer(
            message->seq,
            controller->last_move_sequence
        )
    )
    {
        return ROBOT_STATUS_OLD_SEQUENCE;
    }

    controller->last_move_sequence = message->seq;
    controller->has_last_move_sequence = true;
    controller->yaw_rate_x10 = move.head_yaw_rate_x10;
    controller->pitch_rate_x10 = move.head_pitch_rate_x10;
    controller->command_deadline_ms =
        controller->current_time_ms + move.valid_ms;
    controller->last_update_ms = controller->current_time_ms;
    controller->motion_active = (
        controller->yaw_rate_x10 != 0 ||
        controller->pitch_rate_x10 != 0
    );
    controller->motion_timed_out = false;

    return ROBOT_STATUS_OK;
}


RobotStatusCode RobotMotion_HandleMessage(
    RobotMotionController *controller,
    const RobotProtocolMessage *message
)
{
    RobotStatusCode status;

    if (controller == NULL || message == NULL)
    {
        return ROBOT_STATUS_NOT_IMPLEMENTED;
    }

    if (message->service != ROBOT_SERVICE_MOTION)
    {
        return ROBOT_STATUS_UNKNOWN_SERVICE;
    }

    switch (message->opcode)
    {
        case ROBOT_MOTION_MOVE:
            return handle_move(controller, message);

        case ROBOT_MOTION_STOP:
            status = validate_empty_payload(message);

            if (status == ROBOT_STATUS_OK)
            {
                stop_motion(controller);
                controller->motion_timed_out = false;
            }

            return status;

        case ROBOT_MOTION_CENTER:
            status = validate_empty_payload(message);

            if (
                status == ROBOT_STATUS_OK &&
                controller->estop_latched
            )
            {
                status = ROBOT_STATUS_ESTOP_ACTIVE;
            }

            if (status == ROBOT_STATUS_OK)
            {
                stop_motion(controller);
                controller->yaw_x10 =
                    ROBOT_MOTION_YAW_CENTER_X10;
                controller->pitch_x10 =
                    ROBOT_MOTION_PITCH_CENTER_X10;
                controller->motion_timed_out = false;
                apply_servo_angles(controller);
            }

            return status;

        case ROBOT_MOTION_ESTOP:
            status = validate_empty_payload(message);

            if (status == ROBOT_STATUS_OK)
            {
                stop_motion(controller);
                controller->estop_latched = true;
            }

            return status;

        case ROBOT_MOTION_CLEAR_ESTOP:
            status = validate_empty_payload(message);

            if (status == ROBOT_STATUS_OK)
            {
                controller->estop_latched = false;
                stop_motion(controller);
            }

            return status;

        default:
            return ROBOT_STATUS_UNKNOWN_OPCODE;
    }
}


void RobotMotion_Init(
    RobotMotionController *controller,
    RobotMotionServoHandler servo_handler,
    void *user_context,
    uint32_t now_ms
)
{
    if (controller == NULL)
    {
        return;
    }

    memset(controller, 0, sizeof(*controller));
    controller->servo_handler = servo_handler;
    controller->user_context = user_context;
    controller->yaw_x10 = ROBOT_MOTION_YAW_CENTER_X10;
    controller->pitch_x10 = ROBOT_MOTION_PITCH_CENTER_X10;
    controller->last_applied_yaw = 0xFFU;
    controller->last_applied_pitch = 0xFFU;
    controller->current_time_ms = now_ms;
    controller->last_update_ms = now_ms;

    apply_servo_angles(controller);
}


void RobotMotion_Process(
    RobotMotionController *controller,
    uint32_t now_ms
)
{
    uint32_t integration_end;
    uint32_t elapsed_ms;
    int32_t yaw_total;
    int32_t pitch_total;

    if (controller == NULL)
    {
        return;
    }

    controller->current_time_ms = now_ms;

    if (!controller->motion_active)
    {
        controller->last_update_ms = now_ms;
        return;
    }

    integration_end = now_ms;

    if (time_reached(now_ms, controller->command_deadline_ms))
    {
        integration_end = controller->command_deadline_ms;
    }

    elapsed_ms = integration_end - controller->last_update_ms;
    controller->last_update_ms = integration_end;

    yaw_total =
        (int32_t)controller->yaw_rate_x10 *
        (int32_t)elapsed_ms +
        controller->yaw_remainder;
    pitch_total =
        (int32_t)controller->pitch_rate_x10 *
        (int32_t)elapsed_ms +
        controller->pitch_remainder;

    controller->yaw_x10 += yaw_total / 1000;
    controller->pitch_x10 += pitch_total / 1000;
    controller->yaw_remainder = yaw_total % 1000;
    controller->pitch_remainder = pitch_total % 1000;

    apply_servo_angles(controller);

    if (time_reached(now_ms, controller->command_deadline_ms))
    {
        stop_motion(controller);
        controller->last_update_ms = now_ms;
        controller->motion_timed_out = true;
    }
}


uint8_t RobotMotion_GetYawAngle(
    const RobotMotionController *controller
)
{
    if (controller == NULL)
    {
        return 0U;
    }

    return angle_x10_to_degrees(controller->yaw_x10);
}


uint8_t RobotMotion_GetPitchAngle(
    const RobotMotionController *controller
)
{
    if (controller == NULL)
    {
        return 0U;
    }

    return angle_x10_to_degrees(controller->pitch_x10);
}
