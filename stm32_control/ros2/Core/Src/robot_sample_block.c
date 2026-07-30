#include "robot_sample_block.h"

#include <stddef.h>
#include <string.h>


static void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8U);
}


static void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8U) & 0xFFU);
    data[2] = (uint8_t)((value >> 16U) & 0xFFU);
    data[3] = (uint8_t)(value >> 24U);
}


bool RobotSampleManager_Init(
    RobotSampleManager *manager,
    const RobotSensorRegistry *registry
)
{
    uint8_t index;

    if (manager == NULL || registry == NULL)
    {
        return false;
    }
    memset(manager, 0, sizeof(*manager));

    for (index = 0U; index < registry->count; ++index)
    {
        const RobotSensorDescriptor *descriptor =
            &registry->descriptors[index];
        RobotSampleRing *ring;
        if (
            descriptor->buffer_policy !=
                ROBOT_SENSOR_BUFFER_SAMPLE_RING
        )
        {
            continue;
        }
        if (
            manager->stream_count >=
                ROBOT_SAMPLE_STREAM_CAPACITY ||
            descriptor->sample_size == 0U ||
            descriptor->sample_size > ROBOT_SAMPLE_MAX_BYTES ||
            descriptor->sample_period_ms == 0U
        )
        {
            return false;
        }
        ring = &manager->streams[manager->stream_count++];
        ring->descriptor = descriptor;
        ring->registry_index = index;
    }
    return true;
}


bool RobotSampleManager_Push(
    RobotSampleManager *manager,
    uint8_t stream_index,
    const RobotSensorSample *sample
)
{
    RobotSampleRing *ring;
    uint8_t tail;

    if (
        manager == NULL ||
        sample == NULL ||
        stream_index >= manager->stream_count
    )
    {
        return false;
    }
    ring = &manager->streams[stream_index];
    if (
        sample->length != ring->descriptor->sample_size ||
        sample->length > ROBOT_SAMPLE_MAX_BYTES
    )
    {
        manager->rejected++;
        return false;
    }

    if (ring->count >= ROBOT_SAMPLE_RING_CAPACITY)
    {
        ring->head = (uint8_t)(
            (ring->head + 1U) % ROBOT_SAMPLE_RING_CAPACITY
        );
        ring->count--;
        if (ring->dropped_since_block < 0xFFFFU)
        {
            ring->dropped_since_block++;
        }
        manager->dropped++;
    }

    tail = (uint8_t)(
        (ring->head + ring->count) %
        ROBOT_SAMPLE_RING_CAPACITY
    );
    memcpy(
        ring->samples[tail],
        sample->data,
        sample->length
    );
    ring->timestamps_ms[tail] = sample->timestamp_ms;
    ring->count++;
    manager->pushed++;
    return true;
}


uint8_t RobotSampleManager_Poll(
    RobotSampleManager *manager,
    RobotSensorRegistry *registry,
    uint32_t now_ms
)
{
    uint8_t stream_index;
    uint8_t pushed = 0U;

    if (manager == NULL || registry == NULL)
    {
        return 0U;
    }
    for (
        stream_index = 0U;
        stream_index < manager->stream_count;
        ++stream_index
    )
    {
        RobotSampleRing *ring = &manager->streams[stream_index];
        RobotSensorSample sample;
        if (!registry->running[ring->registry_index])
        {
            continue;
        }
        memset(&sample, 0, sizeof(sample));
        if (
            ring->descriptor->driver.poll_or_read(
                now_ms,
                &sample,
                ring->descriptor->driver_context
            ) &&
            RobotSampleManager_Push(
                manager,
                stream_index,
                &sample
            )
        )
        {
            pushed++;
        }
    }
    return pushed;
}


bool RobotSampleManager_BuildNext(
    RobotSampleManager *manager,
    uint32_t now_ms,
    bool force,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
)
{
    uint8_t stream_index;

    if (output_length != NULL)
    {
        *output_length = 0U;
    }
    if (
        manager == NULL ||
        output == NULL ||
        output_length == NULL ||
        output_capacity < ROBOT_SAMPLE_BLOCK_HEADER_SIZE
    )
    {
        return false;
    }

    for (
        stream_index = 0U;
        stream_index < manager->stream_count;
        ++stream_index
    )
    {
        RobotSampleRing *ring = &manager->streams[stream_index];
        const RobotSensorDescriptor *descriptor = ring->descriptor;
        uint16_t max_samples;
        uint16_t emit_count;
        uint16_t offset = ROBOT_SAMPLE_BLOCK_HEADER_SIZE;
        uint16_t index;

        if (ring->count == 0U)
        {
            continue;
        }
        if (
            !force &&
            ring->count < ROBOT_SAMPLE_RING_CAPACITY &&
            RobotTime_ElapsedMs(
                ring->timestamps_ms[ring->head],
                now_ms
            ) < descriptor->max_latency_ms
        )
        {
            continue;
        }

        max_samples = (uint16_t)(
            (output_capacity - ROBOT_SAMPLE_BLOCK_HEADER_SIZE) /
            descriptor->sample_size
        );
        emit_count = ring->count;
        if (emit_count > max_samples)
        {
            emit_count = max_samples;
        }
        if (emit_count == 0U)
        {
            return false;
        }

        write_u16(output, descriptor->sensor_type);
        output[2] = descriptor->instance_id;
        output[3] = descriptor->schema_version;
        write_u32(output + 4U, ring->timestamps_ms[ring->head]);
        write_u32(output + 8U, descriptor->sample_period_ms);
        write_u16(output + 12U, emit_count);
        write_u16(output + 14U, descriptor->sample_size);
        write_u16(output + 16U, ring->dropped_since_block);

        for (index = 0U; index < emit_count; ++index)
        {
            const uint8_t sample_index = (uint8_t)(
                (ring->head + index) %
                ROBOT_SAMPLE_RING_CAPACITY
            );
            memcpy(
                output + offset,
                ring->samples[sample_index],
                descriptor->sample_size
            );
            offset = (uint16_t)(
                offset + descriptor->sample_size
            );
        }

        ring->head = (uint8_t)(
            (ring->head + emit_count) %
            ROBOT_SAMPLE_RING_CAPACITY
        );
        ring->count = (uint8_t)(ring->count - emit_count);
        ring->dropped_since_block = 0U;
        *output_length = offset;
        manager->blocks_built++;
        manager->samples_built += emit_count;
        return true;
    }
    return false;
}
