#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_dispatcher.h"
#include "robot_motion.h"
#include "robot_protocol.h"
#include "v1_vectors.h"


_Static_assert(
    ROBOT_PROTOCOL_VERSION == V1_GOLDEN_PROTOCOL_VERSION,
    "protocol version differs from canonical vectors"
);
_Static_assert(
    ROBOT_PROTOCOL_HEADER_SIZE == V1_GOLDEN_HEADER_SIZE,
    "header size differs from canonical vectors"
);
_Static_assert(
    ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE ==
        V1_GOLDEN_MAX_PAYLOAD_SIZE,
    "payload limit differs from canonical vectors"
);
_Static_assert(
    ROBOT_KNOWN_FLAGS_MASK == V1_GOLDEN_KNOWN_FLAGS_MASK,
    "flag mask differs from canonical vectors"
);
_Static_assert(
    ROBOT_NODE_LINUX == V1_GOLDEN_NODE_LINUX &&
        ROBOT_NODE_ESP32 == V1_GOLDEN_NODE_ESP32 &&
        ROBOT_NODE_STM32 == V1_GOLDEN_NODE_STM32 &&
        ROBOT_NODE_BROADCAST == V1_GOLDEN_NODE_BROADCAST,
    "node IDs differ from canonical vectors"
);
_Static_assert(
    ROBOT_SERVICE_SYSTEM == V1_GOLDEN_SERVICE_SYSTEM &&
        ROBOT_SERVICE_MOTION == V1_GOLDEN_SERVICE_MOTION &&
        ROBOT_SERVICE_TELEMETRY ==
            V1_GOLDEN_SERVICE_TELEMETRY &&
        ROBOT_SERVICE_CONFIG == V1_GOLDEN_SERVICE_CONFIG &&
        ROBOT_SERVICE_EVENT == V1_GOLDEN_SERVICE_EVENT &&
        ROBOT_SERVICE_OTA == V1_GOLDEN_SERVICE_OTA,
    "service IDs differ from canonical vectors"
);


typedef struct
{
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE];
    uint16_t wire_length;
    uint8_t yaw;
    uint8_t pitch;
    uint32_t servo_updates;
} TestState;


typedef struct
{
    RobotProtocolMessage message;
    uint8_t payload[ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint32_t message_count;
} ProtocolCapture;


static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static uint16_t build_application_frame(
    const uint8_t *message,
    uint16_t message_length,
    uint8_t *wire
);


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
    uint8_t message[ROBOT_PROTOCOL_MAX_MESSAGE_SIZE] = {0};
    uint16_t message_length = (uint16_t)(
        ROBOT_PROTOCOL_HEADER_SIZE + payload_length
    );

    message[0] = ROBOT_PROTOCOL_VERSION;
    message[1] = flags;
    message[2] = ROBOT_NODE_LINUX;
    message[3] = ROBOT_NODE_STM32;
    message[4] = ROBOT_SERVICE_MOTION;
    message[5] = opcode;
    write_u16_le(message + 6U, sequence);
    write_u16_le(message + 8U, payload_length);

    if (payload_length > 0U)
    {
        memcpy(
            message + ROBOT_PROTOCOL_HEADER_SIZE,
            payload,
            payload_length
        );
    }

    return build_application_frame(
        message,
        message_length,
        wire
    );
}


static uint16_t build_application_frame(
    const uint8_t *message,
    uint16_t message_length,
    uint8_t *wire
)
{
    uint8_t raw[ROBOT_PROTOCOL_MAX_RAW_SIZE] = {0};
    uint16_t crc;
    uint16_t encoded_length = 0U;

    assert(message != NULL);
    assert(message_length <= ROBOT_PROTOCOL_MAX_MESSAGE_SIZE);
    memcpy(raw, message, message_length);

    crc = RobotProtocol_Crc16CcittFalse(
        message,
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


static const V1GoldenValidVector *find_valid_vector(
    const char *name
)
{
    size_t index;

    for (index = 0U; index < V1_GOLDEN_VALID_VECTOR_COUNT; ++index)
    {
        if (
            strcmp(
                V1_GOLDEN_VALID_VECTORS[index].name,
                name
            ) == 0
        )
        {
            return &V1_GOLDEN_VALID_VECTORS[index];
        }
    }

    assert(false);
    return NULL;
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


static void capture_message(
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length,
    void *user_context
)
{
    ProtocolCapture *capture = (ProtocolCapture *)user_context;

    assert(message != NULL);
    assert(raw != NULL);
    assert(
        raw_length ==
        ROBOT_PROTOCOL_HEADER_SIZE +
            message->payload_length
    );
    assert(message->payload_length <= sizeof(capture->payload));

    capture->message = *message;

    if (message->payload_length > 0U)
    {
        memcpy(
            capture->payload,
            message->payload,
            message->payload_length
        );
    }

    capture->message.payload = capture->payload;
    ++capture->message_count;
}


static void feed_protocol_wire(
    RobotProtocolContext *context,
    const uint8_t *wire,
    uint16_t length
)
{
    uint16_t index;

    for (index = 0U; index < length; ++index)
    {
        RobotProtocol_InputByte(context, wire[index]);
    }
}


static void feed_wire(
    RobotDispatcher *dispatcher,
    const uint8_t *wire,
    uint16_t length,
    uint32_t now_ms
)
{
    uint16_t index;

    for (index = 0U; index < length; ++index)
    {
        RobotDispatcher_SetTime(
            dispatcher,
            now_ms
        );
        RobotProtocol_InputByte(
            RobotDispatcher_GetProtocol(
                dispatcher
            ),
            wire[index]
        );
    }
}


static void assert_motion_response_wire(
    const TestState *state,
    uint8_t opcode,
    uint16_t sequence,
    RobotStatusCode status
)
{
    uint8_t expected_message[
        ROBOT_PROTOCOL_HEADER_SIZE + 2U
    ] = {
        ROBOT_PROTOCOL_VERSION,
        ROBOT_FLAG_RESPONSE,
        ROBOT_NODE_STM32,
        ROBOT_NODE_LINUX,
        ROBOT_SERVICE_MOTION,
        opcode,
        0U,
        0U,
        2U,
        0U,
        0U,
        0U
    };
    uint8_t expected_wire[
        ROBOT_PROTOCOL_MAX_WIRE_SIZE
    ] = {0};
    uint16_t expected_length;

    if (status != ROBOT_STATUS_OK)
    {
        expected_message[1] |= ROBOT_FLAG_ERROR;
    }

    write_u16_le(expected_message + 6U, sequence);
    write_u16_le(
        expected_message + ROBOT_PROTOCOL_HEADER_SIZE,
        (uint16_t)status
    );

    expected_length = build_application_frame(
        expected_message,
        (uint16_t)sizeof(expected_message),
        expected_wire
    );

    assert(state->wire_length == expected_length);
    assert(
        memcmp(
            state->wire,
            expected_wire,
            expected_length
        ) == 0
    );
}


static void test_crc_reference_vector(void)
{
    assert(
        RobotProtocol_Crc16CcittFalse(
            V1_GOLDEN_CRC_DATA,
            (uint16_t)sizeof(V1_GOLDEN_CRC_DATA)
        ) == V1_GOLDEN_CRC_EXPECTED
    );
}


static void test_golden_codec_vectors(void)
{
    size_t index;

    for (index = 0U; index < V1_GOLDEN_VALID_VECTOR_COUNT; ++index)
    {
        const V1GoldenValidVector *vector =
            &V1_GOLDEN_VALID_VECTORS[index];
        RobotProtocolMessage message;
        RobotProtocolDecodeStatus status;
        uint8_t encoded[ROBOT_PROTOCOL_MAX_MESSAGE_SIZE] = {0};
        uint16_t encoded_length = 0U;

        status = RobotProtocol_DecodeMessage(
            vector->message,
            (uint16_t)vector->message_length,
            &message
        );

        assert(status == ROBOT_DECODE_OK);
        assert(message.version == vector->version);
        assert(message.flags == vector->flags);
        assert(message.src == vector->src);
        assert(message.dst == vector->dst);
        assert(message.service == vector->service);
        assert(message.opcode == vector->opcode);
        assert(message.seq == vector->seq);
        assert(message.payload_length == vector->payload_length);
        assert(
            message.payload ==
            vector->message + ROBOT_PROTOCOL_HEADER_SIZE
        );
        assert(
            RobotProtocol_EncodeMessage(
                &message,
                encoded,
                (uint16_t)sizeof(encoded),
                &encoded_length
            )
        );
        assert(encoded_length == vector->message_length);
        assert(
            memcmp(
                encoded,
                vector->message,
                vector->message_length
            ) == 0
        );
    }
}


static void test_codec_argument_validation(void)
{
    const V1GoldenValidVector *vector =
        find_valid_vector("linux_motion_move");
    RobotProtocolMessage message;
    uint8_t encoded[ROBOT_PROTOCOL_MAX_MESSAGE_SIZE] = {0};
    uint16_t encoded_length = 123U;

    assert(
        RobotProtocol_DecodeMessage(
            NULL,
            0U,
            &message
        ) == ROBOT_DECODE_NULL_DATA
    );
    assert(
        RobotProtocol_DecodeMessage(
            vector->message,
            (uint16_t)vector->message_length,
            NULL
        ) == ROBOT_DECODE_NULL_DATA
    );
    assert(
        strcmp(
            RobotProtocol_DecodeStatusName(
                (RobotProtocolDecodeStatus)255
            ),
            "UNKNOWN_STATUS"
        ) == 0
    );

    assert(
        RobotProtocol_DecodeMessage(
            vector->message,
            (uint16_t)vector->message_length,
            &message
        ) == ROBOT_DECODE_OK
    );

    assert(
        !RobotProtocol_EncodeMessage(
            &message,
            encoded,
            (uint16_t)(vector->message_length - 1U),
            &encoded_length
        )
    );
    assert(encoded_length == 0U);

    message.flags = (uint8_t)(ROBOT_KNOWN_FLAGS_MASK | 0x10U);
    encoded_length = 123U;
    assert(
        !RobotProtocol_EncodeMessage(
            &message,
            encoded,
            (uint16_t)sizeof(encoded),
            &encoded_length
        )
    );
    assert(encoded_length == 0U);
}


static void test_golden_valid_ingress_vectors(void)
{
    size_t index;

    for (index = 0U; index < V1_GOLDEN_VALID_VECTOR_COUNT; ++index)
    {
        const V1GoldenValidVector *vector =
            &V1_GOLDEN_VALID_VECTORS[index];
        RobotProtocolContext context;
        ProtocolCapture capture = {0};
        uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
        uint16_t wire_length;

        if (
            (vector->src != ROBOT_NODE_LINUX &&
             vector->src != ROBOT_NODE_ESP32) ||
            (vector->dst != ROBOT_NODE_STM32 &&
             vector->dst != ROBOT_NODE_BROADCAST)
        )
        {
            continue;
        }

        RobotProtocol_Init(
            &context,
            NULL,
            capture_message,
            &capture
        );
        wire_length = build_application_frame(
            vector->message,
            (uint16_t)vector->message_length,
            wire
        );
        feed_protocol_wire(&context, wire, wire_length);

        assert(capture.message_count == 1U);
        assert(capture.message.version == vector->version);
        assert(capture.message.flags == vector->flags);
        assert(capture.message.src == vector->src);
        assert(capture.message.dst == vector->dst);
        assert(capture.message.service == vector->service);
        assert(capture.message.opcode == vector->opcode);
        assert(capture.message.seq == vector->seq);
        assert(capture.message.payload_length == vector->payload_length);
        assert(
            memcmp(
                capture.message.payload,
                vector->message + ROBOT_PROTOCOL_HEADER_SIZE,
                vector->payload_length
            ) == 0
        );
    }
}


static void test_golden_invalid_vectors_are_rejected(void)
{
    size_t index;

    for (index = 0U; index < V1_GOLDEN_INVALID_VECTOR_COUNT; ++index)
    {
        const V1GoldenInvalidVector *vector =
            &V1_GOLDEN_INVALID_VECTORS[index];
        RobotProtocolContext context;
        ProtocolCapture capture = {0};
        RobotProtocolMessage message;
        RobotProtocolDecodeStatus status;
        uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
        uint16_t wire_length;

        status = RobotProtocol_DecodeMessage(
            vector->message,
            (uint16_t)vector->message_length,
            &message
        );

        assert(status != ROBOT_DECODE_OK);
        assert(
            strcmp(
                RobotProtocol_DecodeStatusName(status),
                vector->expected_error
            ) == 0
        );

        RobotProtocol_Init(
            &context,
            NULL,
            capture_message,
            &capture
        );
        wire_length = build_application_frame(
            vector->message,
            (uint16_t)vector->message_length,
            wire
        );
        feed_protocol_wire(&context, wire, wire_length);

        assert(capture.message_count == 0U);
        assert(context.format_error_count == 1U);
    }
}


static void test_oversized_frame_resynchronizes(void)
{
    const V1GoldenValidVector *vector =
        find_valid_vector("linux_motion_move");
    RobotProtocolContext context;
    ProtocolCapture capture = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;
    uint16_t index;

    RobotProtocol_Init(
        &context,
        NULL,
        capture_message,
        &capture
    );

    for (
        index = 0U;
        index <= ROBOT_PROTOCOL_MAX_COBS_SIZE;
        ++index
    )
    {
        RobotProtocol_InputByte(&context, 0x7FU);
    }

    assert(context.dropping_oversized_frame);
    assert(context.overflow_error_count == 1U);

    wire_length = build_application_frame(
        vector->message,
        (uint16_t)vector->message_length,
        wire
    );
    feed_protocol_wire(&context, wire, wire_length);
    assert(!context.dropping_oversized_frame);
    assert(capture.message_count == 0U);

    feed_protocol_wire(&context, wire, wire_length);
    assert(capture.message_count == 1U);
}


static void test_truncated_frame_is_rejected(void)
{
    const V1GoldenValidVector *vector =
        find_valid_vector("linux_motion_move");
    RobotProtocolContext context;
    ProtocolCapture capture = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotProtocol_Init(
        &context,
        NULL,
        capture_message,
        &capture
    );
    wire_length = build_application_frame(
        vector->message,
        (uint16_t)vector->message_length,
        wire
    );

    assert(wire_length > 2U);
    wire[wire_length - 2U] = 0U;
    feed_protocol_wire(
        &context,
        wire,
        (uint16_t)(wire_length - 1U)
    );

    assert(capture.message_count == 0U);
    assert(
        context.crc_error_count +
        context.format_error_count >= 1U
    );
}


static void test_move_timeout_sequence_and_center(void)
{
    RobotMotionController controller;
    RobotDispatcher dispatcher;
    TestState state = {0};
    uint8_t payload[12] = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotMotion_Init(
        &controller,
        capture_servos,
        &state,
        0U
    );
    RobotDispatcher_Init(
        &dispatcher,
        &controller,
        capture_transmit,
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
    feed_wire(&dispatcher, wire, wire_length, 0U);
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
    feed_wire(&dispatcher, wire, wire_length, 310U);
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
    feed_wire(&dispatcher, wire, wire_length, 510U);

    assert(state.yaw == 90U);
    assert(state.pitch == 90U);
    assert(state.wire_length > 0U);
    assert(state.wire[state.wire_length - 1U] == 0U);
}


static void test_stm32_can_send_telemetry_through_esp32(void)
{
    const V1GoldenValidVector *expected =
        find_valid_vector("stm32_telemetry_sample");
    RobotMotionController controller;
    RobotDispatcher dispatcher;
    TestState state = {0};
    uint8_t payload[] = {0x34U, 0x12U};
    uint8_t raw[ROBOT_PROTOCOL_MAX_RAW_SIZE] = {0};
    uint16_t raw_length = 0U;

    RobotMotion_Init(
        &controller,
        capture_servos,
        &state,
        0U
    );
    RobotDispatcher_Init(
        &dispatcher,
        &controller,
        capture_transmit,
        &state,
        0U
    );

    assert(
        RobotProtocol_SendMessage(
            RobotDispatcher_GetProtocol(&dispatcher),
            0U,
            ROBOT_NODE_LINUX,
            ROBOT_SERVICE_TELEMETRY,
            0x01U,
            RobotProtocol_NextSequence(
                RobotDispatcher_GetProtocol(
                    &dispatcher
                )
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
    assert(
        raw_length ==
        expected->message_length + ROBOT_PROTOCOL_CRC_SIZE
    );
    assert(
        memcmp(
            raw,
            expected->message,
            expected->message_length
        ) == 0
    );
}


static void test_corrupt_frame_is_rejected(void)
{
    RobotMotionController controller;
    RobotDispatcher dispatcher;
    TestState state = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotMotion_Init(
        &controller,
        capture_servos,
        &state,
        0U
    );
    RobotDispatcher_Init(
        &dispatcher,
        &controller,
        capture_transmit,
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
    feed_wire(&dispatcher, wire, wire_length, 0U);

    assert(
        dispatcher.protocol.crc_error_count +
        dispatcher.protocol.format_error_count >= 1U
    );
    assert(state.wire_length == 0U);
}


static void test_dispatcher_rejects_unknown_service(void)
{
    RobotMotionController controller;
    RobotDispatcher dispatcher;
    TestState state = {0};
    RobotProtocolMessage message = {0};
    uint8_t raw[ROBOT_PROTOCOL_HEADER_SIZE] = {
        ROBOT_PROTOCOL_VERSION,
        ROBOT_FLAG_ACK_REQUIRED,
        ROBOT_NODE_LINUX,
        ROBOT_NODE_STM32,
        ROBOT_SERVICE_TELEMETRY,
        0x7FU,
        0x44U,
        0x33U,
        0x00U,
        0x00U
    };
    uint8_t response_raw[
        ROBOT_PROTOCOL_MAX_RAW_SIZE
    ] = {0};
    uint16_t response_raw_length = 0U;
    RobotProtocolMessage response;

    RobotMotion_Init(
        &controller,
        capture_servos,
        &state,
        0U
    );
    RobotDispatcher_Init(
        &dispatcher,
        &controller,
        capture_transmit,
        &state,
        0U
    );

    assert(
        RobotProtocol_DecodeMessage(
            raw,
            (uint16_t)sizeof(raw),
            &message
        ) == ROBOT_DECODE_OK
    );
    assert(
        RobotDispatcher_OnMessage(
            &dispatcher,
            &message,
            raw,
            (uint16_t)sizeof(raw)
        ) ==
        ROBOT_STATUS_UNKNOWN_SERVICE
    );
    assert(dispatcher.dispatched_message_count == 1U);
    assert(dispatcher.unsupported_service_count == 1U);
    assert(state.wire_length > 1U);

    assert(
        cobs_decode(
            state.wire,
            (uint16_t)(state.wire_length - 1U),
            response_raw,
            &response_raw_length
        )
    );
    assert(
        response_raw_length ==
        ROBOT_PROTOCOL_HEADER_SIZE +
            2U +
            ROBOT_PROTOCOL_CRC_SIZE
    );
    assert(
        RobotProtocol_DecodeMessage(
            response_raw,
            (uint16_t)(
                response_raw_length -
                ROBOT_PROTOCOL_CRC_SIZE
            ),
            &response
        ) == ROBOT_DECODE_OK
    );
    assert(response.src == ROBOT_NODE_STM32);
    assert(response.dst == ROBOT_NODE_LINUX);
    assert(response.service == ROBOT_SERVICE_TELEMETRY);
    assert(response.opcode == 0x7FU);
    assert(response.seq == 0x3344U);
    assert(
        response.flags ==
        (
            ROBOT_FLAG_RESPONSE |
            ROBOT_FLAG_ERROR
        )
    );
    assert(response.payload_length == 2U);
    assert(
        response.payload[0] ==
        ROBOT_STATUS_UNKNOWN_SERVICE
    );
    assert(response.payload[1] == 0U);
}


static void test_all_motion_response_bytes_are_stable(void)
{
    RobotMotionController controller;
    RobotDispatcher dispatcher;
    TestState state = {0};
    uint8_t wire[ROBOT_PROTOCOL_MAX_WIRE_SIZE] = {0};
    uint16_t wire_length;

    RobotMotion_Init(
        &controller,
        capture_servos,
        &state,
        0U
    );
    RobotDispatcher_Init(
        &dispatcher,
        &controller,
        capture_transmit,
        &state,
        0U
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_STOP,
        ROBOT_FLAG_ACK_REQUIRED,
        10U,
        NULL,
        0U,
        wire
    );
    feed_wire(&dispatcher, wire, wire_length, 1U);
    assert_motion_response_wire(
        &state,
        ROBOT_MOTION_STOP,
        10U,
        ROBOT_STATUS_OK
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_ESTOP,
        ROBOT_FLAG_ACK_REQUIRED,
        11U,
        NULL,
        0U,
        wire
    );
    feed_wire(&dispatcher, wire, wire_length, 2U);
    assert(controller.estop_latched);
    assert_motion_response_wire(
        &state,
        ROBOT_MOTION_ESTOP,
        11U,
        ROBOT_STATUS_OK
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_CENTER,
        ROBOT_FLAG_ACK_REQUIRED,
        12U,
        NULL,
        0U,
        wire
    );
    feed_wire(&dispatcher, wire, wire_length, 3U);
    assert_motion_response_wire(
        &state,
        ROBOT_MOTION_CENTER,
        12U,
        ROBOT_STATUS_ESTOP_ACTIVE
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_CLEAR_ESTOP,
        ROBOT_FLAG_ACK_REQUIRED,
        13U,
        NULL,
        0U,
        wire
    );
    feed_wire(&dispatcher, wire, wire_length, 4U);
    assert(!controller.estop_latched);
    assert_motion_response_wire(
        &state,
        ROBOT_MOTION_CLEAR_ESTOP,
        13U,
        ROBOT_STATUS_OK
    );

    wire_length = build_linux_frame(
        ROBOT_MOTION_CENTER,
        ROBOT_FLAG_ACK_REQUIRED,
        14U,
        NULL,
        0U,
        wire
    );
    feed_wire(&dispatcher, wire, wire_length, 5U);
    assert_motion_response_wire(
        &state,
        ROBOT_MOTION_CENTER,
        14U,
        ROBOT_STATUS_OK
    );
}


int main(void)
{
    test_crc_reference_vector();
    test_golden_codec_vectors();
    test_codec_argument_validation();
    test_golden_valid_ingress_vectors();
    test_golden_invalid_vectors_are_rejected();
    test_oversized_frame_resynchronizes();
    test_truncated_frame_is_rejected();
    test_move_timeout_sequence_and_center();
    test_stm32_can_send_telemetry_through_esp32();
    test_corrupt_frame_is_rejected();
    test_dispatcher_rejects_unknown_service();
    test_all_motion_response_bytes_are_stable();

    puts("STM32 V1 protocol and motion host tests passed");
    return 0;
}
