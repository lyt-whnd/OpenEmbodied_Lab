#include "robot_sensor.h"

#include <stddef.h>
#include <string.h>


bool RobotSensorRegistry_Init(
    RobotSensorRegistry *registry,
    const RobotSensorDescriptor *descriptors,
    uint8_t count
)
{
    uint8_t index;
    uint8_t other;

    if (
        registry == NULL ||
        (count > 0U && descriptors == NULL) ||
        count > ROBOT_SENSOR_MAX_REGISTERED
    )
    {
        return false;
    }

    memset(registry, 0, sizeof(*registry));

    for (index = 0U; index < count; ++index)
    {
        const RobotSensorDescriptor *descriptor = &descriptors[index];
        if (
            descriptor->schema_version == 0U ||
            descriptor->sample_size == 0U ||
            descriptor->sample_size > ROBOT_SENSOR_MAX_SAMPLE_BYTES ||
            descriptor->driver.init == NULL ||
            descriptor->driver.start == NULL ||
            descriptor->driver.stop == NULL ||
            descriptor->driver.configure == NULL ||
            descriptor->driver.poll_or_read == NULL ||
            descriptor->driver.get_info == NULL
        )
        {
            return false;
        }

        for (other = 0U; other < index; ++other)
        {
            if (
                descriptors[other].sensor_type ==
                    descriptor->sensor_type &&
                descriptors[other].instance_id ==
                    descriptor->instance_id
            )
            {
                return false;
            }
        }
    }

    registry->descriptors = descriptors;
    registry->count = count;

    for (index = 0U; index < count; ++index)
    {
        registry->initialized[index] =
            descriptors[index].driver.init(
                descriptors[index].driver_context
            );
    }

    return true;
}


const RobotSensorDescriptor *RobotSensorRegistry_Find(
    const RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id,
    uint8_t *index
)
{
    uint8_t candidate;

    if (registry == NULL)
    {
        return NULL;
    }

    for (candidate = 0U; candidate < registry->count; ++candidate)
    {
        const RobotSensorDescriptor *descriptor =
            &registry->descriptors[candidate];
        if (
            descriptor->sensor_type == sensor_type &&
            descriptor->instance_id == instance_id
        )
        {
            if (index != NULL)
            {
                *index = candidate;
            }
            return descriptor;
        }
    }
    return NULL;
}


RobotStatusCode RobotSensorRegistry_Start(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id
)
{
    uint8_t index;
    const RobotSensorDescriptor *descriptor =
        RobotSensorRegistry_Find(
            registry, sensor_type, instance_id, &index
        );
    if (descriptor == NULL)
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }
    if (!registry->initialized[index])
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }
    if (registry->running[index])
    {
        return ROBOT_STATUS_OK;
    }
    if (!descriptor->driver.start(descriptor->driver_context))
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }
    registry->running[index] = true;
    return ROBOT_STATUS_OK;
}


RobotStatusCode RobotSensorRegistry_Stop(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id
)
{
    uint8_t index;
    const RobotSensorDescriptor *descriptor =
        RobotSensorRegistry_Find(
            registry, sensor_type, instance_id, &index
        );
    if (descriptor == NULL)
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }
    if (registry->running[index])
    {
        descriptor->driver.stop(descriptor->driver_context);
        registry->running[index] = false;
    }
    return ROBOT_STATUS_OK;
}


RobotStatusCode RobotSensorRegistry_Configure(
    RobotSensorRegistry *registry,
    uint16_t sensor_type,
    uint8_t instance_id,
    uint16_t key,
    int32_t value
)
{
    uint8_t index;
    const RobotSensorDescriptor *descriptor =
        RobotSensorRegistry_Find(
            registry, sensor_type, instance_id, &index
        );
    if (
        descriptor == NULL ||
        !registry->initialized[index]
    )
    {
        return ROBOT_STATUS_DEVICE_OFFLINE;
    }
    return descriptor->driver.configure(
        key,
        value,
        descriptor->driver_context
    );
}
