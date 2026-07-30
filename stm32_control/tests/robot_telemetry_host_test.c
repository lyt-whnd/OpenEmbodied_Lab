#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_telemetry.h"


static RobotSensorDescriptor descriptor(
    uint16_t type,
    uint8_t instance,
    uint16_t sample_size,
    uint16_t max_latency
)
{
    RobotSensorDescriptor result;
    memset(&result, 0, sizeof(result));
    result.sensor_type = type;
    result.instance_id = instance;
    result.schema_version = 1U;
    result.default_qos = ROBOT_QOS_BEST_EFFORT;
    result.buffer_policy = ROBOT_SENSOR_BUFFER_LATEST;
    result.sample_size = sample_size;
    result.sample_period_ms = max_latency;
    result.max_latency_ms = max_latency;
    return result;
}


static RobotSensorSample sample(
    uint32_t timestamp,
    uint16_t sequence,
    uint16_t length,
    uint8_t value
)
{
    RobotSensorSample result;
    memset(&result, 0, sizeof(result));
    result.timestamp_ms = timestamp;
    result.sample_sequence = sequence;
    result.length = length;
    memset(result.data, value, length);
    return result;
}


static uint32_t read_u32(const uint8_t *data)
{
    return (
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8U) |
        ((uint32_t)data[2] << 16U) |
        ((uint32_t)data[3] << 24U)
    );
}


static void test_async_sources_and_sample_timestamp(void)
{
    RobotTelemetryAggregator aggregator;
    RobotSensorDescriptor slow = descriptor(1U, 0U, 1U, 1000U);
    RobotSensorDescriptor fast = descriptor(2U, 0U, 1U, 20U);
    RobotSensorSample slow_sample = sample(100U, 1U, 1U, 0x11U);
    RobotSensorSample fast_sample = sample(130U, 2U, 1U, 0x22U);
    uint8_t batch[ROBOT_TELEMETRY_BATCH_MAX_BYTES];
    uint16_t length = 0U;

    RobotTelemetry_Init(&aggregator, 100U);
    assert(RobotTelemetry_PublishLatest(
        &aggregator, &slow, &slow_sample, 100U
    ));
    assert(!RobotTelemetry_ShouldFlush(&aggregator, 119U));
    assert(RobotTelemetry_PublishLatest(
        &aggregator, &fast, &fast_sample, 130U
    ));
    assert(RobotTelemetry_ShouldFlush(&aggregator, 150U));
    assert(RobotTelemetry_BuildBatch(
        &aggregator,
        150U,
        false,
        batch,
        sizeof(batch),
        &length
    ));
    assert(batch[1] == 2U);
    assert(read_u32(batch + 2U + 4U) == 100U);
    assert(read_u32(batch + 15U + 4U) == 130U);
}


static void test_latest_replacement_and_size_boundaries(void)
{
    RobotTelemetryAggregator aggregator;
    RobotSensorDescriptor descriptors[
        ROBOT_TELEMETRY_LATEST_CAPACITY
    ];
    RobotSensorSample samples[
        ROBOT_TELEMETRY_LATEST_CAPACITY
    ];
    uint8_t batch[ROBOT_TELEMETRY_BATCH_MAX_BYTES];
    uint16_t length = 0U;
    uint8_t index;

    RobotTelemetry_Init(&aggregator, 0U);
    for (
        index = 0U;
        index < ROBOT_TELEMETRY_LATEST_CAPACITY;
        ++index
    )
    {
        const uint16_t size = index == 5U ? 32U : 30U;
        descriptors[index] = descriptor(
            (uint16_t)(index + 1U),
            0U,
            size,
            1000U
        );
        samples[index] = sample(
            index,
            index,
            size,
            index
        );
        assert(RobotTelemetry_PublishLatest(
            &aggregator,
            &descriptors[index],
            &samples[index],
            index
        ));
    }
    assert(RobotTelemetry_ShouldFlush(&aggregator, 6U));
    assert(RobotTelemetry_BuildBatch(
        &aggregator,
        6U,
        false,
        batch,
        sizeof(batch),
        &length
    ));
    assert(length == ROBOT_TELEMETRY_BATCH_MAX_BYTES);
    assert(batch[1] == 6U);

    RobotTelemetry_Init(&aggregator, 0U);
    assert(RobotTelemetry_PublishLatest(
        &aggregator,
        &descriptors[0],
        &samples[0],
        0U
    ));
    samples[0].data[0] = 0xEEU;
    assert(RobotTelemetry_PublishLatest(
        &aggregator,
        &descriptors[0],
        &samples[0],
        1U
    ));
    assert(aggregator.replaced == 1U);
    assert(!RobotTelemetry_BuildBatch(
        &aggregator,
        1U,
        true,
        batch,
        ROBOT_SENSOR_RECORD_HEADER_SIZE + 31U,
        &length
    ));
    assert(RobotTelemetry_BuildBatch(
        &aggregator,
        1U,
        true,
        batch,
        sizeof(batch),
        &length
    ));
    assert(batch[ROBOT_TELEMETRY_BATCH_HEADER_SIZE +
                 ROBOT_SENSOR_RECORD_HEADER_SIZE] == 0xEEU);
}


static void test_oversize_and_emergency_bypass(void)
{
    RobotTelemetryAggregator aggregator;
    RobotSensorDescriptor invalid = descriptor(
        1U, 0U, ROBOT_SENSOR_MAX_SAMPLE_BYTES, 10U
    );
    RobotSensorSample oversized = sample(
        0U, 0U, ROBOT_SENSOR_MAX_SAMPLE_BYTES, 0U
    );

    RobotTelemetry_Init(&aggregator, 0U);
    oversized.length = ROBOT_SENSOR_MAX_SAMPLE_BYTES + 1U;
    assert(!RobotTelemetry_PublishLatest(
        &aggregator, &invalid, &oversized, 0U
    ));
    assert(aggregator.rejected_oversize == 1U);
    assert(RobotTelemetry_RequiresImmediateSend(
        ROBOT_SENSOR_BUFFER_EVENT,
        ROBOT_PRIORITY_NORMAL
    ));
    assert(RobotTelemetry_RequiresImmediateSend(
        ROBOT_SENSOR_BUFFER_LATEST,
        ROBOT_PRIORITY_EMERGENCY
    ));
}


int main(void)
{
    test_async_sources_and_sample_timestamp();
    test_latest_replacement_and_size_boundaries();
    test_oversize_and_emergency_bypass();
    puts("STM32 telemetry batch host tests passed");
    return 0;
}
