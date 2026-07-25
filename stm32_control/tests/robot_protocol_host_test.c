#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_motion.h"
#include "robot_protocol.h"


typedef struct
{
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE];
    uint16_t wire_length;
    uint8_t yaw;
    uint8_t pitch;
    uint32_t servo_updates;
} TestState;


static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static bool cobs_encode(
    const uint8_t *input,
    uint16_t input_length,
    uint8_t *output,
    uint16_t *output_length
)
{
    uint16_t read_index = 0U;
    uint16_t write_index = 1U;
    uint16_t code_index = 0U;
    uint8_t code = 1U;

    while (read_index < input_length)
    {
        if (input[read_index] == 0U)
        {
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
            ++read_index;
            continue;
        }

        output[write_index++] = input[read_index++];
        ++code;

        if (code == 0xFFU)
        {
            output[code_index] = code;
            code = 1U;
            code_index = write_index++;
        }
    }

    output[code_index] = code;
    *output_length = write_index;
    return true;
}


static bool cobs_decode(
    const uint8_t *input,
    uint16_t input_length,
    uint8_t *output,
    uint16_t *output_length
)
{
    uint16_t read_index = 0U;
    uint16_t write_index = 0U;

    while (read_index < input_length)
    {
        uint8_t code = input[read_index++];
        uint8_t offset;

        if (code == 0U)
        {
            return false;
        }

        for (offset = 1U; offset < code; ++offset)
        {
            if (read_index >= input_length)
            {
                return false;
            }

            output[write_index++] = input[read_index++];
        }

        if (code != 0xFFU && read_index < input_length)
        {
            output[write_index++] = 0U;
        }
    }

    *output_length = write_index;
    return true;
}


static uint16_t build_linux_frame(
    uint8_t opcode,
    uint8_t flags,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length,
    uint8_t *wire
)
{
    uint8_t raw[ROBOT_PROTOCOL_MAX_RAW_SIZE] = {0};
    uint16_t message_length = (uint16_t)(
        ROBOT_PROTOCOL_HEADER_SIZE + payload_length
    );
    uint16_t crc;
    uint16_t encoded_length = 0U;

    raw[0] = ROBOT_PROTOCOL_VERSION;
    raw[1] = flags;
    raw[2] = ROBOT_NODE_LINUX;
    raw[3] = ROBOT_NODE_STM32;
    raw[4] = ROBOT_SERVICE_MOTION;
    raw[5] = opcode;
    write_u16_le(raw + 6U, sequence);
    write_u16_le(raw + 8U, payload_length);

    if (payload_length > 0U)
    {
        memcpy(
            raw + ROBOT_PROTOCOL_HEADER_SIZE,
            payload,
            payload_length
        );
    }

    crc = RobotProtocol_Crc16CcittFalse(
        raw,
        message_length
    );
    write_u16_le(raw + message_length, crc);

    assert(
        cobs_encode(
            raw,
            (uint16_t)(
                message_length + ROBOT_PROTOCOL_CRC_SIZE
            ),
            wire,
            &encoded_length
        )
    );

    wire[encoded_length] = 0U;
    return (uint16_t)(encoded_length + 1U);
}


static bool capture_transmit(
    const uint8_t *data,
    uint16_t length,
    void *user_context
)
{
    TestState *state = (TestState *)user_context;

    assert(length <= sizeof(state->wire));
    memcpy(state->wire, data, length);
    state->wire_length = length;
    return true;
}


static void capture_servos(
    uint8_t yaw,
    uint8_t pitch,
    void *user_context
)
{
    TestState *state = (TestState *)user_context;

    state->yaw = yaw;
    state->pitch = pitch;
    ++state->servo_updates;
}


static void feed_wire(
    RobotMotionController *controller,
    const uint8_t *wire,
    uint16_t length,
    uint32_t now_ms
)
{
    uint16_t index;

    for (index = 0U; index < length; ++index)
    {
        RobotMotion_InputByte(
            controller,
            wire[index],
            now_ms
        );
    }
}


static void test_crc_reference_vector(void)
{
    static const uint8_t reference[] = {
        '1', '2', '3', '4', '5',
        '6', '7', '8', '9'
    };

    assert(
        RobotProtocol_Crc16CcittFalse(
            reference,
            (uint16_t)sizeof(reference)
        ) == 0x29B1U
    );
}


static void test_move_timeout_sequence_and_center(void)
{
    RobotMotionController controller;
    TestState state = {0};
    uint8_t payload[12] = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotMotion_Init(
        &controller,
        capture_transmit,
        capture_servos,
        &state,
        0U
    );

    assert(state.yaw == 90U);
    assert(state.pitch == 90U);

    write_u16_le(payload, 77U);
    write_u16_le(payload + 2U, 300U);
    write_u16_le(payload + 8U, 300U);

    wire_length = build_linux_frame(
        ROBOT_MOTION_MOVE,
        ROBOT_FLAG_REALTIME,
        1U,
        payload,
        (uint16_t)sizeof(payload),
        wire
    );
    feed_wire(&controller, wire, wire_length, 0U);
    RobotMotion_Process(&controller, 100U);

    assert(state.yaw == 93U);
    assert(state.pitch == 90U);
    assert(controller.motion_active);

    RobotMotion_Process(&controller, 300U);
    assert(state.yaw == 99U);
    assert(!controller.motion_active);
    assert(controller.motion_timed_out);

    /*
     * 同 epoch 的重复 MOVE 序号必须被忽略。
     */
    write_u16_le(payload + 8U, (uint16_t)(int16_t)-300);
    wire_length = build_linux_frame(
        ROBOT_MOTION_MOVE,
        ROBOT_FLAG_REALTIME,
        1U,
        payload,
        (uint16_t)sizeof(payload),
        wire
    );
    feed_wire(&controller, wire, wire_length, 310U);
    RobotMotion_Process(&controller, 500U);
    assert(state.yaw == 99U);

    wire_length = build_linux_frame(
        ROBOT_MOTION_CENTER,
        ROBOT_FLAG_ACK_REQUIRED,
        2U,
        NULL,
        0U,
        wire
    );
    feed_wire(&controller, wire, wire_length, 510U);

    assert(state.yaw == 90U);
    assert(state.pitch == 90U);
    assert(state.wire_length > 0U);
    assert(state.wire[state.wire_length - 1U] == 0U);
}


static void test_stm32_can_send_telemetry_through_esp32(void)
{
    RobotMotionController controller;
    TestState state = {0};
    uint8_t payload[] = {0x34U, 0x12U};
    uint8_t raw[ROBOT_PROTOCOL_MAX_RAW_SIZE] = {0};
    uint16_t raw_length = 0U;

    RobotMotion_Init(
        &controller,
        capture_transmit,
        capture_servos,
        &state,
        0U
    );

    assert(
        RobotProtocol_SendMessage(
            RobotMotion_GetProtocol(&controller),
            0U,
            ROBOT_NODE_LINUX,
            ROBOT_SERVICE_TELEMETRY,
            0x01U,
            RobotProtocol_NextSequence(
                RobotMotion_GetProtocol(&controller)
            ),
            payload,
            (uint16_t)sizeof(payload)
        )
    );
    assert(state.wire_length > 1U);
    assert(
        cobs_decode(
            state.wire,
            (uint16_t)(state.wire_length - 1U),
            raw,
            &raw_length
        )
    );
    assert(raw_length == 14U);
    assert(raw[0] == ROBOT_PROTOCOL_VERSION);
    assert(raw[2] == ROBOT_NODE_STM32);
    assert(raw[3] == ROBOT_NODE_LINUX);
    assert(raw[4] == ROBOT_SERVICE_TELEMETRY);
    assert(raw[10] == 0x34U);
    assert(raw[11] == 0x12U);
}


static void test_corrupt_frame_is_rejected(void)
{
    RobotMotionController controller;
    TestState state = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotMotion_Init(
        &controller,
        capture_transmit,
        capture_servos,
        &state,
        0U
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_STOP,
        ROBOT_FLAG_ACK_REQUIRED,
        5U,
        NULL,
        0U,
        wire
    );
    wire[3] ^= 0x01U;
    feed_wire(&controller, wire, wire_length, 0U);

    assert(
        controller.protocol.crc_error_count +
        controller.protocol.format_error_count >= 1U
    );
    assert(state.wire_length == 0U);
}


int main(void)
{
    test_crc_reference_vector();
    test_move_timeout_sequence_and_center();
    test_stm32_can_send_telemetry_through_esp32();
    test_corrupt_frame_is_rejected();

    puts("STM32 V1 protocol and motion host tests passed");
    return 0;
}
