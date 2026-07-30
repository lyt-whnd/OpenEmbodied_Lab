#include "robot_reliable.h"

#include <stddef.h>


static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8U)
    );
}


static uint32_t read_u32_le(const uint8_t *data)
{
    return
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U);
}


static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static void write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)(value >> 24U);
}


bool RobotReliable_DecodeRequest(
    const RobotProtocolMessage *message,
    RobotReliableRequest *request
)
{
    if (
        message == NULL ||
        request == NULL ||
        message->payload == NULL ||
        message->payload_length <
            ROBOT_RELIABLE_REQUEST_SIZE ||
        message->payload[0] !=
            ROBOT_RELIABLE_SCHEMA_VERSION
    )
    {
        return false;
    }

    request->epoch = read_u16_le(message->payload + 1U);
    request->request_id =
        read_u32_le(message->payload + 3U);
    request->payload =
        message->payload + ROBOT_RELIABLE_REQUEST_SIZE;
    request->payload_length = (uint16_t)(
        message->payload_length -
        ROBOT_RELIABLE_REQUEST_SIZE
    );

    return true;
}


bool RobotReliable_SendResult(
    RobotProtocolContext *protocol,
    const RobotProtocolMessage *request_message,
    const RobotReliableRequest *request,
    RobotResultStage stage,
    RobotStatusCode status
)
{
    uint8_t payload[ROBOT_RELIABLE_RESULT_SIZE] = {0};
    uint8_t flags = ROBOT_FLAG_RESPONSE;

    if (
        protocol == NULL ||
        request_message == NULL ||
        request == NULL
    )
    {
        return false;
    }

    payload[0] = ROBOT_RELIABLE_SCHEMA_VERSION;
    write_u16_le(payload + 1U, request->epoch);
    write_u32_le(payload + 3U, request->request_id);
    payload[7] = (uint8_t)stage;
    write_u16_le(payload + 8U, (uint16_t)status);

    if (
        stage == ROBOT_RESULT_STAGE_FAILED ||
        status != ROBOT_STATUS_OK
    )
    {
        flags |= ROBOT_FLAG_ERROR;
    }

    return RobotProtocol_SendMessage(
        protocol,
        flags,
        request_message->src,
        request_message->service,
        request_message->opcode,
        request_message->seq,
        payload,
        (uint16_t)sizeof(payload)
    );
}
