#include "robot_sensor_service.h"

#include <stddef.h>


static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8U)
    );
}


static int32_t read_i32(const uint8_t *data)
{
    return (int32_t)(
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U)
    );
}


static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static bool read_target(
    const RobotProtocolMessage *message,
    RobotSensorTarget *target
)
{
    if (
        message->payload == NULL ||
        message->payload_length < ROBOT_SENSOR_TARGET_SIZE
    )
    {
        return false;
    }
    target->sensor_type = read_u16(message->payload);
    target->instance_id = message->payload[2];
    return true;
}


static bool send_response(
    RobotSensorService *service,
    const RobotProtocolMessage *request,
    const uint8_t *payload,
    uint16_t payload_length
)
{
    return RobotProtocol_SendMessage(
        service->protocol,
        ROBOT_FLAG_RESPONSE,
        request->src,
        ROBOT_SERVICE_SENSOR,
        request->opcode,
        request->seq,
        payload,
        payload_length
    );
}


static RobotStatusCode handle_list(
    RobotSensorService *service,
    const RobotProtocolMessage *message
)
{
    uint8_t payload[
        2U + ROBOT_SENSOR_MAX_REGISTERED * 4U
    ];
    uint8_t index;
    uint16_t offset = 2U;

    if (message->payload_length != 0U)
    {
        return ROBOT_STATUS_BAD_LENGTH;
    }

    payload[0] = ROBOT_SENSOR_SCHEMA_VERSION;
    payload[1] = service->registry->count;

    for (index = 0U; index < service->registry->count; ++index)
    {
        const RobotSensorDescriptor *descriptor =
            &service->registry->descriptors[index];
        write_u16(payload + offset, descriptor->sensor_type);
        payload[offset + 2U] = descriptor->instance_id;
        payload[offset + 3U] = descriptor->schema_version;
        offset = (uint16_t)(offset + 4U);
    }

    return send_response(service, message, payload, offset)
        ? ROBOT_STATUS_OK
        : ROBOT_STATUS_DEVICE_OFFLINE;
}


static RobotStatusCode handle_info(
    RobotSensorService *service,
    const RobotProtocolMessage *message
)
{
    RobotSensorTarget target;
    const RobotSensorDescriptor *descriptor;
    uint8_t payload[64U];
    uint16_t driver_info_length = 0U;

    if (
        message->payload_length != ROBOT_SENSOR_TARGET_SIZE ||
        !read_target(message, &target)
    )
    {
        return ROBOT_STATUS_BAD_LENGTH;
    }

    descriptor = RobotSensorRegistry_Find(
        service->registry,
        target.sensor_type,
        target.instance_id,
        NULL
    );
    if (descriptor == NULL)
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }

    payload[0] = ROBOT_SENSOR_SCHEMA_VERSION;
    write_u16(payload + 1U, descriptor->sensor_type);
    payload[3] = descriptor->instance_id;
    payload[4] = descriptor->schema_version;
    payload[5] = (uint8_t)descriptor->default_qos;
    payload[6] = (uint8_t)descriptor->buffer_policy;
    write_u16(payload + 7U, descriptor->sample_size);
    write_u16(payload + 9U, descriptor->sample_period_ms);
    write_u16(payload + 11U, descriptor->max_latency_ms);
    descriptor->driver.get_info(
        payload + 13U,
        (uint16_t)(sizeof(payload) - 13U),
        &driver_info_length,
        descriptor->driver_context
    );
    if (driver_info_length > sizeof(payload) - 13U)
    {
        return ROBOT_STATUS_BAD_LENGTH;
    }

    return send_response(
        service,
        message,
        payload,
        (uint16_t)(13U + driver_info_length)
    )
        ? ROBOT_STATUS_OK
        : ROBOT_STATUS_DEVICE_OFFLINE;
}


void RobotSensorService_Init(
    RobotSensorService *service,
    RobotSensorRegistry *registry,
    RobotProtocolContext *protocol
)
{
    if (service == NULL)
    {
        return;
    }
    service->registry = registry;
    service->protocol = protocol;
}


RobotStatusCode RobotSensorService_Handle(
    RobotSensorService *service,
    const RobotProtocolMessage *message
)
{
    RobotSensorTarget target;

    if (
        service == NULL ||
        service->registry == NULL ||
        service->protocol == NULL ||
        message == NULL
    )
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }

    switch (message->opcode)
    {
        case ROBOT_SENSOR_LIST:
            return handle_list(service, message);
        case ROBOT_SENSOR_INFO:
            return handle_info(service, message);
        case ROBOT_SENSOR_CONFIG:
            if (
                message->payload_length != 9U ||
                !read_target(message, &target)
            )
            {
                return ROBOT_STATUS_BAD_LENGTH;
            }
            return RobotSensorRegistry_Configure(
                service->registry,
                target.sensor_type,
                target.instance_id,
                read_u16(message->payload + 3U),
                read_i32(message->payload + 5U)
            );
        case ROBOT_SENSOR_START:
            if (
                message->payload_length != ROBOT_SENSOR_TARGET_SIZE ||
                !read_target(message, &target)
            )
            {
                return ROBOT_STATUS_BAD_LENGTH;
            }
            return RobotSensorRegistry_Start(
                service->registry,
                target.sensor_type,
                target.instance_id
            );
        case ROBOT_SENSOR_STOP:
            if (
                message->payload_length != ROBOT_SENSOR_TARGET_SIZE ||
                !read_target(message, &target)
            )
            {
                return ROBOT_STATUS_BAD_LENGTH;
            }
            return RobotSensorRegistry_Stop(
                service->registry,
                target.sensor_type,
                target.instance_id
            );
        default:
            return ROBOT_STATUS_UNKNOWN_OPCODE;
    }
}
