#include "robot_telemetry.h"

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


static RobotTelemetryLatestSlot *find_slot(
    RobotTelemetryAggregator *aggregator,
    uint16_t sensor_type,
    uint8_t instance_id
)
{
    uint8_t index;
    RobotTelemetryLatestSlot *free_slot = NULL;
    for (
        index = 0U;
        index < ROBOT_TELEMETRY_LATEST_CAPACITY;
        ++index
    )
    {
        RobotTelemetryLatestSlot *slot =
            &aggregator->latest[index];
        if (
            slot->pending &&
            slot->sensor_type == sensor_type &&
            slot->instance_id == instance_id
        )
        {
            return slot;
        }
        if (!slot->pending && free_slot == NULL)
        {
            free_slot = slot;
        }
    }
    return free_slot;
}


static uint16_t pending_encoded_size(
    const RobotTelemetryAggregator *aggregator
)
{
    uint16_t result = ROBOT_TELEMETRY_BATCH_HEADER_SIZE;
    uint8_t index;
    for (
        index = 0U;
        index < ROBOT_TELEMETRY_LATEST_CAPACITY;
        ++index
    )
    {
        const RobotTelemetryLatestSlot *slot =
            &aggregator->latest[index];
        if (slot->pending)
        {
            result = (uint16_t)(
                result +
                ROBOT_SENSOR_RECORD_HEADER_SIZE +
                slot->data_length
            );
        }
    }
    return result;
}


void RobotTelemetry_Init(
    RobotTelemetryAggregator *aggregator,
    uint32_t now_ms
)
{
    if (aggregator == NULL)
    {
        return;
    }
    memset(aggregator, 0, sizeof(*aggregator));
    aggregator->last_flush_ms = now_ms;
}


bool RobotTelemetry_PublishLatest(
    RobotTelemetryAggregator *aggregator,
    const RobotSensorDescriptor *descriptor,
    const RobotSensorSample *sample,
    uint32_t now_ms
)
{
    RobotTelemetryLatestSlot *slot;

    if (
        aggregator == NULL ||
        descriptor == NULL ||
        sample == NULL ||
        sample->length == 0U ||
        sample->length > ROBOT_SENSOR_MAX_SAMPLE_BYTES ||
        sample->length != descriptor->sample_size
    )
    {
        if (aggregator != NULL)
        {
            aggregator->rejected_oversize++;
        }
        return false;
    }

    slot = find_slot(
        aggregator,
        descriptor->sensor_type,
        descriptor->instance_id
    );
    if (slot == NULL)
    {
        aggregator->rejected_full++;
        return false;
    }
    if (slot->pending)
    {
        aggregator->replaced++;
    }

    slot->sensor_type = descriptor->sensor_type;
    slot->instance_id = descriptor->instance_id;
    slot->schema_version = descriptor->schema_version;
    slot->timestamp_ms = sample->timestamp_ms;
    slot->sample_sequence = sample->sample_sequence;
    slot->data_length = sample->length;
    slot->max_latency_ms = descriptor->max_latency_ms;
    slot->published_at_ms = now_ms;
    memcpy(slot->data, sample->data, sample->length);
    slot->pending = true;
    aggregator->published++;
    return true;
}


uint8_t RobotTelemetry_PollRegistry(
    RobotTelemetryAggregator *aggregator,
    RobotSensorRegistry *registry,
    uint32_t now_ms
)
{
    uint8_t index;
    uint8_t published = 0U;

    if (aggregator == NULL || registry == NULL)
    {
        return 0U;
    }

    for (index = 0U; index < registry->count; ++index)
    {
        const RobotSensorDescriptor *descriptor =
            &registry->descriptors[index];
        RobotSensorSample sample;
        if (
            !registry->running[index] ||
            descriptor->buffer_policy !=
                ROBOT_SENSOR_BUFFER_LATEST
        )
        {
            continue;
        }
        memset(&sample, 0, sizeof(sample));
        if (
            descriptor->driver.poll_or_read(
                now_ms,
                &sample,
                descriptor->driver_context
            ) &&
            RobotTelemetry_PublishLatest(
                aggregator,
                descriptor,
                &sample,
                now_ms
            )
        )
        {
            published++;
        }
    }
    return published;
}


bool RobotTelemetry_ShouldFlush(
    const RobotTelemetryAggregator *aggregator,
    uint32_t now_ms
)
{
    uint8_t index;
    bool has_pending = false;

    if (aggregator == NULL)
    {
        return false;
    }

    if (
        pending_encoded_size(aggregator) >=
        ROBOT_TELEMETRY_BATCH_MAX_BYTES -
        ROBOT_SENSOR_RECORD_HEADER_SIZE -
        ROBOT_SENSOR_MAX_SAMPLE_BYTES
    )
    {
        return true;
    }

    for (
        index = 0U;
        index < ROBOT_TELEMETRY_LATEST_CAPACITY;
        ++index
    )
    {
        const RobotTelemetryLatestSlot *slot =
            &aggregator->latest[index];
        if (!slot->pending)
        {
            continue;
        }
        has_pending = true;
        if (
            slot->max_latency_ms == 0U ||
            RobotTime_ElapsedMs(
                slot->published_at_ms,
                now_ms
            ) >= slot->max_latency_ms
        )
        {
            return true;
        }
    }

    return (
        has_pending &&
        RobotTime_ElapsedMs(
            aggregator->last_flush_ms,
            now_ms
        ) >= ROBOT_TELEMETRY_FLUSH_PERIOD_MS
    );
}


bool RobotTelemetry_BuildBatch(
    RobotTelemetryAggregator *aggregator,
    uint32_t now_ms,
    bool force,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
)
{
    uint8_t index;
    uint8_t count = 0U;
    uint16_t offset = ROBOT_TELEMETRY_BATCH_HEADER_SIZE;

    if (output_length != NULL)
    {
        *output_length = 0U;
    }
    if (
        aggregator == NULL ||
        output == NULL ||
        output_length == NULL ||
        output_capacity < ROBOT_TELEMETRY_BATCH_HEADER_SIZE ||
        output_capacity > ROBOT_TELEMETRY_BATCH_MAX_BYTES ||
        (!force && !RobotTelemetry_ShouldFlush(
            aggregator, now_ms
        ))
    )
    {
        return false;
    }

    output[0] = ROBOT_TELEMETRY_BATCH_SCHEMA_VERSION;
    output[1] = 0U;

    for (
        index = 0U;
        index < ROBOT_TELEMETRY_LATEST_CAPACITY;
        ++index
    )
    {
        RobotTelemetryLatestSlot *slot =
            &aggregator->latest[index];
        const uint16_t record_size = (uint16_t)(
            ROBOT_SENSOR_RECORD_HEADER_SIZE +
            slot->data_length
        );
        if (!slot->pending)
        {
            continue;
        }
        if ((uint16_t)(output_capacity - offset) < record_size)
        {
            continue;
        }

        write_u16(output + offset, slot->sensor_type);
        output[offset + 2U] = slot->instance_id;
        output[offset + 3U] = slot->schema_version;
        write_u32(output + offset + 4U, slot->timestamp_ms);
        write_u16(
            output + offset + 8U,
            slot->sample_sequence
        );
        write_u16(
            output + offset + 10U,
            slot->data_length
        );
        memcpy(
            output + offset + ROBOT_SENSOR_RECORD_HEADER_SIZE,
            slot->data,
            slot->data_length
        );
        offset = (uint16_t)(offset + record_size);
        slot->pending = false;
        count++;
    }

    if (count == 0U)
    {
        return false;
    }

    output[1] = count;
    *output_length = offset;
    aggregator->last_flush_ms = now_ms;
    aggregator->batches_built++;
    aggregator->records_built += count;
    return true;
}


bool RobotTelemetry_RequiresImmediateSend(
    RobotSensorBufferPolicy buffer_policy,
    RobotPriority priority
)
{
    return (
        buffer_policy == ROBOT_SENSOR_BUFFER_EVENT ||
        priority == ROBOT_PRIORITY_EMERGENCY
    );
}
