#include "robot_protocol.h"

#include <string.h>


static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8U)
    );
}


static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)read_u16_le(data);
}


static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


uint16_t RobotProtocol_Crc16CcittFalse(
    const uint8_t *data,
    uint16_t length
)
{
    uint16_t crc = 0xFFFFU;
    uint16_t index;
    uint8_t bit;

    if (data == NULL)
    {
        return crc;
    }

    for (index = 0U; index < length; ++index)
    {
        crc ^= (uint16_t)((uint16_t)data[index] << 8U);

        for (bit = 0U; bit < 8U; ++bit)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}


static bool cobs_encode(
    const uint8_t *input,
    uint16_t input_length,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
)
{
    uint16_t read_index = 0U;
    uint16_t write_index = 1U;
    uint16_t code_index = 0U;
    uint8_t code = 1U;

    if (
        input == NULL ||
        output == NULL ||
        output_length == NULL ||
        output_capacity == 0U
    )
    {
        return false;
    }

    while (read_index < input_length)
    {
        if (input[read_index] == 0U)
        {
            output[code_index] = code;
            code = 1U;
            code_index = write_index;

            if (write_index >= output_capacity)
            {
                return false;
            }

            ++write_index;
            ++read_index;
            continue;
        }

        if (write_index >= output_capacity)
        {
            return false;
        }

        output[write_index] = input[read_index];
        ++write_index;
        ++read_index;
        ++code;

        if (code == 0xFFU)
        {
            output[code_index] = code;
            code = 1U;
            code_index = write_index;

            if (write_index >= output_capacity)
            {
                return false;
            }

            ++write_index;
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
    uint16_t output_capacity,
    uint16_t *output_length
)
{
    uint16_t read_index = 0U;
    uint16_t write_index = 0U;

    if (
        input == NULL ||
        output == NULL ||
        output_length == NULL ||
        input_length == 0U
    )
    {
        return false;
    }

    while (read_index < input_length)
    {
        uint8_t code = input[read_index];
        uint8_t offset;

        if (code == 0U)
        {
            return false;
        }

        ++read_index;

        for (offset = 1U; offset < code; ++offset)
        {
            if (
                read_index >= input_length ||
                write_index >= output_capacity
            )
            {
                return false;
            }

            output[write_index] = input[read_index];
            ++write_index;
            ++read_index;
        }

        if (
            code != 0xFFU &&
            read_index < input_length
        )
        {
            if (write_index >= output_capacity)
            {
                return false;
            }

            output[write_index] = 0U;
            ++write_index;
        }
    }

    *output_length = write_index;
    return true;
}


static bool source_is_valid(uint8_t source)
{
    return (
        source == ROBOT_NODE_LINUX ||
        source == ROBOT_NODE_ESP32
    );
}


static bool destination_is_valid(uint8_t destination)
{
    return (
        destination == ROBOT_NODE_STM32 ||
        destination == ROBOT_NODE_BROADCAST
    );
}


static void process_encoded_frame(
    RobotProtocolContext *context
)
{
    uint16_t raw_length = 0U;
    uint16_t message_length;
    uint16_t received_crc;
    uint16_t expected_crc;
    uint16_t payload_length;
    RobotProtocolMessage message;

    if (
        !cobs_decode(
            context->rx_encoded,
            context->rx_encoded_length,
            context->rx_raw,
            (uint16_t)sizeof(context->rx_raw),
            &raw_length
        )
    )
    {
        ++context->format_error_count;
        return;
    }

    if (
        raw_length <
        ROBOT_PROTOCOL_HEADER_SIZE + ROBOT_PROTOCOL_CRC_SIZE
    )
    {
        ++context->format_error_count;
        return;
    }

    message_length = (uint16_t)(
        raw_length - ROBOT_PROTOCOL_CRC_SIZE
    );
    received_crc = read_u16_le(
        context->rx_raw + message_length
    );
    expected_crc = RobotProtocol_Crc16CcittFalse(
        context->rx_raw,
        message_length
    );

    if (received_crc != expected_crc)
    {
        ++context->crc_error_count;
        return;
    }

    payload_length = read_u16_le(context->rx_raw + 8U);

    if (
        context->rx_raw[0] != ROBOT_PROTOCOL_VERSION ||
        (context->rx_raw[1] & ~ROBOT_KNOWN_FLAGS_MASK) != 0U ||
        !source_is_valid(context->rx_raw[2]) ||
        !destination_is_valid(context->rx_raw[3]) ||
        payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE ||
        message_length !=
            ROBOT_PROTOCOL_HEADER_SIZE + payload_length
    )
    {
        ++context->format_error_count;
        return;
    }

    message.version = context->rx_raw[0];
    message.flags = context->rx_raw[1];
    message.src = context->rx_raw[2];
    message.dst = context->rx_raw[3];
    message.service = context->rx_raw[4];
    message.opcode = context->rx_raw[5];
    message.seq = read_u16_le(context->rx_raw + 6U);
    message.payload_length = payload_length;
    message.payload = context->rx_raw + ROBOT_PROTOCOL_HEADER_SIZE;

    ++context->valid_frame_count;

    if (context->message_handler != NULL)
    {
        context->message_handler(
            &message,
            context->user_context
        );
    }
}


void RobotProtocol_Init(
    RobotProtocolContext *context,
    RobotProtocolTransmitHandler transmit_handler,
    RobotProtocolMessageHandler message_handler,
    void *user_context
)
{
    if (context == NULL)
    {
        return;
    }

    memset(context, 0, sizeof(*context));
    context->transmit_handler = transmit_handler;
    context->message_handler = message_handler;
    context->user_context = user_context;
}


void RobotProtocol_InputByte(
    RobotProtocolContext *context,
    uint8_t byte
)
{
    if (context == NULL)
    {
        return;
    }

    if (byte == 0U)
    {
        if (context->dropping_oversized_frame)
        {
            context->dropping_oversized_frame = false;
            context->rx_encoded_length = 0U;
            return;
        }

        if (context->rx_encoded_length > 0U)
        {
            process_encoded_frame(context);
        }

        context->rx_encoded_length = 0U;
        return;
    }

    if (context->dropping_oversized_frame)
    {
        return;
    }

    if (
        context->rx_encoded_length >=
        ROBOT_PROTOCOL_MAX_COBS_SIZE
    )
    {
        context->rx_encoded_length = 0U;
        context->dropping_oversized_frame = true;
        ++context->overflow_error_count;
        return;
    }

    context->rx_encoded[
        context->rx_encoded_length
    ] = byte;
    ++context->rx_encoded_length;
}


uint16_t RobotProtocol_NextSequence(
    RobotProtocolContext *context
)
{
    uint16_t sequence;

    if (context == NULL)
    {
        return 0U;
    }

    sequence = context->next_tx_sequence;
    ++context->next_tx_sequence;
    return sequence;
}


bool RobotProtocol_SendMessage(
    RobotProtocolContext *context,
    uint8_t flags,
    uint8_t dst,
    uint8_t service,
    uint8_t opcode,
    uint16_t sequence,
    const uint8_t *payload,
    uint16_t payload_length
)
{
    uint16_t message_length;
    uint16_t raw_length;
    uint16_t encoded_length = 0U;
    uint16_t crc;

    if (
        context == NULL ||
        context->transmit_handler == NULL ||
        (flags & ~ROBOT_KNOWN_FLAGS_MASK) != 0U ||
        payload_length > ROBOT_PROTOCOL_MAX_PAYLOAD_SIZE ||
        (payload_length > 0U && payload == NULL)
    )
    {
        return false;
    }

    context->tx_message[0] = ROBOT_PROTOCOL_VERSION;
    context->tx_message[1] = flags;
    context->tx_message[2] = ROBOT_NODE_STM32;
    context->tx_message[3] = dst;
    context->tx_message[4] = service;
    context->tx_message[5] = opcode;
    write_u16_le(context->tx_message + 6U, sequence);
    write_u16_le(context->tx_message + 8U, payload_length);

    if (payload_length > 0U)
    {
        memcpy(
            context->tx_message + ROBOT_PROTOCOL_HEADER_SIZE,
            payload,
            payload_length
        );
    }

    message_length = (uint16_t)(
        ROBOT_PROTOCOL_HEADER_SIZE + payload_length
    );
    memcpy(
        context->tx_raw,
        context->tx_message,
        message_length
    );

    crc = RobotProtocol_Crc16CcittFalse(
        context->tx_message,
        message_length
    );
    write_u16_le(context->tx_raw + message_length, crc);
    raw_length = (uint16_t)(
        message_length + ROBOT_PROTOCOL_CRC_SIZE
    );

    if (
        !cobs_encode(
            context->tx_raw,
            raw_length,
            context->tx_wire,
            ROBOT_PROTOCOL_MAX_COBS_SIZE,
            &encoded_length
        )
    )
    {
        return false;
    }

    context->tx_wire[encoded_length] = 0U;

    return context->transmit_handler(
        context->tx_wire,
        (uint16_t)(encoded_length + 1U),
        context->user_context
    );
}


bool RobotProtocol_SendResponse(
    RobotProtocolContext *context,
    const RobotProtocolMessage *request,
    RobotStatusCode status
)
{
    uint8_t payload[2];
    uint8_t flags = ROBOT_FLAG_RESPONSE;

    if (context == NULL || request == NULL)
    {
        return false;
    }

    if (status != ROBOT_STATUS_OK)
    {
        flags |= ROBOT_FLAG_ERROR;
    }

    write_u16_le(payload, (uint16_t)status);

    return RobotProtocol_SendMessage(
        context,
        flags,
        request->src,
        request->service,
        request->opcode,
        request->seq,
        payload,
        (uint16_t)sizeof(payload)
    );
}


bool RobotProtocol_DecodeMotionMove(
    const RobotProtocolMessage *message,
    RobotMotionMovePayload *move
)
{
    if (
        message == NULL ||
        move == NULL ||
        message->service != ROBOT_SERVICE_MOTION ||
        message->opcode != ROBOT_MOTION_MOVE ||
        message->payload_length != 12U ||
        message->payload == NULL
    )
    {
        return false;
    }

    move->control_epoch = read_u16_le(message->payload);
    move->valid_ms = read_u16_le(message->payload + 2U);
    move->linear_mm_s = read_i16_le(message->payload + 4U);
    move->angular_mrad_s = read_i16_le(message->payload + 6U);
    move->head_yaw_rate_x10 = read_i16_le(
        message->payload + 8U
    );
    move->head_pitch_rate_x10 = read_i16_le(
        message->payload + 10U
    );

    return true;
}
