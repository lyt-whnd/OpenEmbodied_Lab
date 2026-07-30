#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_sample_block.h"


static uint16_t read_u16(const uint8_t *data)
{
    return (uint16_t)(
        (uint16_t)data[0] |
        ((uint16_t)data[1] << 8U)
    );
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


static RobotSensorDescriptor sample_descriptor(
    uint16_t period_ms
)
{
    RobotSensorDescriptor descriptor;
    memset(&descriptor, 0, sizeof(descriptor));
    descriptor.sensor_type = 2U;
    descriptor.schema_version = 1U;
    descriptor.default_qos = ROBOT_QOS_BEST_EFFORT;
    descriptor.buffer_policy =
        ROBOT_SENSOR_BUFFER_SAMPLE_RING;
    descriptor.sample_size = 2U;
    descriptor.sample_period_ms = period_ms;
    descriptor.max_latency_ms = 50U;
    return descriptor;
}


static void test_ring_wrap_drop_count_and_block(void)
{
    RobotSensorDescriptor descriptor = sample_descriptor(10U);
    RobotSensorRegistry registry;
    RobotSampleManager manager;
    RobotSensorSample sample;
    uint8_t output[256U];
    uint16_t length = 0U;
    uint8_t index;

    memset(&registry, 0, sizeof(registry));
    registry.descriptors = &descriptor;
    registry.count = 1U;
    assert(RobotSampleManager_Init(&manager, &registry));

    for (
        index = 0U;
        index < ROBOT_SAMPLE_RING_CAPACITY + 2U;
        ++index
    )
    {
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_ms = (uint32_t)index * 10U;
        sample.length = 2U;
        sample.data[0] = index;
        assert(RobotSampleManager_Push(
            &manager, 0U, &sample
        ));
    }

    assert(manager.dropped == 2U);
    assert(RobotSampleManager_BuildNext(
        &manager,
        100U,
        false,
        output,
        sizeof(output),
        &length
    ));
    assert(read_u32(output + 4U) == 20U);
    assert(read_u32(output + 8U) == 10U);
    assert(read_u16(output + 12U) == ROBOT_SAMPLE_RING_CAPACITY);
    assert(read_u16(output + 14U) == 2U);
    assert(read_u16(output + 16U) == 2U);
    assert(output[ROBOT_SAMPLE_BLOCK_HEADER_SIZE] == 2U);
    assert(length == (
        ROBOT_SAMPLE_BLOCK_HEADER_SIZE +
        ROBOT_SAMPLE_RING_CAPACITY * 2U
    ));
    assert(manager.streams[0].count == 0U);
}


static void test_partial_output_and_period_variant(void)
{
    RobotSensorDescriptor descriptor = sample_descriptor(4U);
    RobotSensorRegistry registry;
    RobotSampleManager manager;
    RobotSensorSample sample;
    uint8_t output[ROBOT_SAMPLE_BLOCK_HEADER_SIZE + 4U];
    uint16_t length = 0U;
    uint8_t index;

    memset(&registry, 0, sizeof(registry));
    registry.descriptors = &descriptor;
    registry.count = 1U;
    assert(RobotSampleManager_Init(&manager, &registry));
    for (index = 0U; index < 3U; ++index)
    {
        memset(&sample, 0, sizeof(sample));
        sample.timestamp_ms = 100U + index * 4U;
        sample.length = 2U;
        sample.data[0] = index;
        assert(RobotSampleManager_Push(
            &manager, 0U, &sample
        ));
    }
    assert(RobotSampleManager_BuildNext(
        &manager,
        120U,
        true,
        output,
        sizeof(output),
        &length
    ));
    assert(read_u32(output + 8U) == 4U);
    assert(read_u16(output + 12U) == 2U);
    assert(manager.streams[0].count == 1U);
}


static void test_invalid_sample_size_rejected(void)
{
    RobotSensorDescriptor descriptor = sample_descriptor(10U);
    RobotSensorRegistry registry;
    RobotSampleManager manager;
    RobotSensorSample sample;

    memset(&registry, 0, sizeof(registry));
    registry.descriptors = &descriptor;
    registry.count = 1U;
    assert(RobotSampleManager_Init(&manager, &registry));
    memset(&sample, 0, sizeof(sample));
    sample.length = 1U;
    assert(!RobotSampleManager_Push(
        &manager, 0U, &sample
    ));
    assert(manager.rejected == 1U);
}


int main(void)
{
    test_ring_wrap_drop_count_and_block();
    test_partial_output_and_period_variant();
    test_invalid_sample_size_rejected();
    puts("STM32 sample block host tests passed");
    return 0;
}
