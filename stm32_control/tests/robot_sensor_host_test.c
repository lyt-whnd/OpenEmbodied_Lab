#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_sensor_service.h"


typedef struct
{
    bool initialized;
    bool running;
    uint16_t config_key;
    int32_t config_value;
} FakeSensor;

static uint32_t transmit_count;


static bool fake_init(void *context)
{
    ((FakeSensor *)context)->initialized = true;
    return true;
}

static bool fake_start(void *context)
{
    ((FakeSensor *)context)->running = true;
    return true;
}

static void fake_stop(void *context)
{
    ((FakeSensor *)context)->running = false;
}

static RobotStatusCode fake_configure(
    uint16_t key,
    int32_t value,
    void *context
)
{
    FakeSensor *sensor = (FakeSensor *)context;
    sensor->config_key = key;
    sensor->config_value = value;
    return ROBOT_STATUS_OK;
}

static bool fake_poll(
    uint32_t now_ms,
    RobotSensorSample *sample,
    void *context
)
{
    (void)context;
    sample->timestamp_ms = now_ms;
    sample->length = 1U;
    sample->data[0] = 0x5AU;
    return true;
}

static void fake_info(
    uint8_t *output,
    uint16_t capacity,
    uint16_t *length,
    void *context
)
{
    (void)context;
    if (capacity > 0U)
    {
        output[0] = 0xA5U;
        *length = 1U;
    }
}

static bool capture_transmit(
    const uint8_t *data,
    uint16_t length,
    void *context
)
{
    (void)context;
    assert(data != NULL);
    assert(length > 0U);
    transmit_count++;
    return true;
}

static void ignore_message(
    const RobotProtocolMessage *message,
    const uint8_t *raw,
    uint16_t raw_length,
    void *context
)
{
    (void)message;
    (void)raw;
    (void)raw_length;
    (void)context;
}

static RobotSensorDriver fake_driver(void)
{
    RobotSensorDriver driver = {
        fake_init,
        fake_start,
        fake_stop,
        fake_configure,
        fake_poll,
        fake_info
    };
    return driver;
}


static void test_second_sensor_only_extends_catalog(void)
{
    FakeSensor first = {0};
    FakeSensor second = {0};
    RobotSensorDescriptor descriptors[2] = {
        {
            ROBOT_SENSOR_TYPE_VIRTUAL_COUNTER,
            0U,
            1U,
            ROBOT_QOS_BEST_EFFORT,
            ROBOT_SENSOR_BUFFER_LATEST,
            1U,
            10U,
            100U,
            {0},
            &first
        },
        {
            ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
            0U,
            2U,
            ROBOT_QOS_BEST_EFFORT,
            ROBOT_SENSOR_BUFFER_SAMPLE_RING,
            1U,
            5U,
            20U,
            {0},
            &second
        }
    };
    RobotSensorRegistry registry;
    descriptors[0].driver = fake_driver();
    descriptors[1].driver = fake_driver();

    assert(RobotSensorRegistry_Init(
        &registry, descriptors, 2U
    ));
    assert(first.initialized && second.initialized);
    assert(RobotSensorRegistry_Find(
        &registry,
        ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
        0U,
        NULL
    ) == &descriptors[1]);
    assert(RobotSensorRegistry_Start(
        &registry,
        ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
        0U
    ) == ROBOT_STATUS_OK);
    assert(second.running);
    assert(RobotSensorRegistry_Configure(
        &registry,
        ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
        0U,
        7U,
        -25
    ) == ROBOT_STATUS_OK);
    assert(second.config_key == 7U);
    assert(second.config_value == -25);
    assert(RobotSensorRegistry_Stop(
        &registry,
        ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
        0U
    ) == ROBOT_STATUS_OK);
    assert(!second.running);
}


static void test_list_info_and_control_service(void)
{
    FakeSensor fake = {0};
    RobotSensorDescriptor descriptor = {
        ROBOT_SENSOR_TYPE_VIRTUAL_COUNTER,
        0U,
        1U,
        ROBOT_QOS_BEST_EFFORT,
        ROBOT_SENSOR_BUFFER_LATEST,
        1U,
        10U,
        100U,
        {0},
        &fake
    };
    RobotSensorRegistry registry;
    RobotProtocolContext protocol;
    RobotSensorService service;
    RobotProtocolMessage request = {
        ROBOT_PROTOCOL_VERSION,
        0U,
        ROBOT_NODE_LINUX,
        ROBOT_NODE_STM32,
        ROBOT_SERVICE_SENSOR,
        ROBOT_SENSOR_LIST,
        1U,
        0U,
        NULL
    };
    uint8_t target[] = {
        (uint8_t)ROBOT_SENSOR_TYPE_VIRTUAL_COUNTER,
        0U,
        0U
    };

    descriptor.driver = fake_driver();
    assert(RobotSensorRegistry_Init(
        &registry, &descriptor, 1U
    ));
    RobotProtocol_Init(
        &protocol,
        capture_transmit,
        ignore_message,
        NULL
    );
    RobotSensorService_Init(&service, &registry, &protocol);
    transmit_count = 0U;

    assert(RobotSensorService_Handle(
        &service, &request
    ) == ROBOT_STATUS_OK);
    assert(transmit_count == 1U);

    request.opcode = ROBOT_SENSOR_INFO;
    request.payload = target;
    request.payload_length = sizeof(target);
    assert(RobotSensorService_Handle(
        &service, &request
    ) == ROBOT_STATUS_OK);
    assert(transmit_count == 2U);

    request.opcode = ROBOT_SENSOR_START;
    assert(RobotSensorService_Handle(
        &service, &request
    ) == ROBOT_STATUS_OK);
    assert(fake.running);
}


int main(void)
{
    test_second_sensor_only_extends_catalog();
    test_list_info_and_control_service();
    puts("STM32 sensor registry/service host tests passed");
    return 0;
}
