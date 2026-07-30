#include "robot_sensor_catalog.h"

#include <stddef.h>


typedef struct
{
    bool running;
    uint16_t sequence;
} VirtualCounterContext;


static VirtualCounterContext counter_context;

typedef struct
{
    bool running;
    uint16_t sequence;
    uint32_t last_sample_ms;
} VirtualSampleContext;

static VirtualSampleContext sample_context;


static bool virtual_init(void *context)
{
    VirtualCounterContext *state = (VirtualCounterContext *)context;
    state->running = false;
    state->sequence = 0U;
    return true;
}


static bool virtual_start(void *context)
{
    ((VirtualCounterContext *)context)->running = true;
    return true;
}


static void virtual_stop(void *context)
{
    ((VirtualCounterContext *)context)->running = false;
}


static RobotStatusCode virtual_configure(
    uint16_t key,
    int32_t value,
    void *context
)
{
    (void)key;
    (void)value;
    (void)context;
    return ROBOT_STATUS_NOT_IMPLEMENTED;
}


static bool virtual_poll(
    uint32_t now_ms,
    RobotSensorSample *sample,
    void *context
)
{
    VirtualCounterContext *state = (VirtualCounterContext *)context;
    if (!state->running || sample == NULL)
    {
        return false;
    }
    sample->timestamp_ms = now_ms;
    sample->sample_sequence = state->sequence++;
    sample->length = 4U;
    sample->data[0] = (uint8_t)(now_ms & 0xFFU);
    sample->data[1] = (uint8_t)((now_ms >> 8U) & 0xFFU);
    sample->data[2] = (uint8_t)((now_ms >> 16U) & 0xFFU);
    sample->data[3] = (uint8_t)(now_ms >> 24U);
    return true;
}


static void virtual_get_info(
    uint8_t *output,
    uint16_t capacity,
    uint16_t *length,
    void *context
)
{
    (void)output;
    (void)capacity;
    (void)context;
    if (length != NULL)
    {
        *length = 0U;
    }
}


static bool sample_init(void *context)
{
    VirtualSampleContext *state = (VirtualSampleContext *)context;
    state->running = false;
    state->sequence = 0U;
    state->last_sample_ms = 0U;
    return true;
}


static bool sample_start(void *context)
{
    ((VirtualSampleContext *)context)->running = true;
    return true;
}


static void sample_stop(void *context)
{
    ((VirtualSampleContext *)context)->running = false;
}


static bool sample_poll(
    uint32_t now_ms,
    RobotSensorSample *sample,
    void *context
)
{
    VirtualSampleContext *state = (VirtualSampleContext *)context;
    uint16_t value;
    if (
        !state->running ||
        sample == NULL ||
        (uint32_t)(now_ms - state->last_sample_ms) < 10U
    )
    {
        return false;
    }
    state->last_sample_ms = now_ms;
    value = state->sequence++;
    sample->timestamp_ms = now_ms;
    sample->sample_sequence = value;
    sample->length = 2U;
    sample->data[0] = (uint8_t)(value & 0xFFU);
    sample->data[1] = (uint8_t)(value >> 8U);
    return true;
}


/*
 * This is the only production registration list. Adding a real sensor means
 * adding its driver/schema and one descriptor here; registry/service/transport
 * code remains unchanged.
 */
static const RobotSensorDescriptor sensor_catalog[] =
{
    {
        ROBOT_SENSOR_TYPE_VIRTUAL_COUNTER,
        0U,
        1U,
        ROBOT_QOS_BEST_EFFORT,
        ROBOT_SENSOR_BUFFER_LATEST,
        4U,
        1000U,
        1000U,
        {
            virtual_init,
            virtual_start,
            virtual_stop,
            virtual_configure,
            virtual_poll,
            virtual_get_info
        },
        &counter_context
    },
    {
        ROBOT_SENSOR_TYPE_VIRTUAL_LEVEL,
        0U,
        1U,
        ROBOT_QOS_BEST_EFFORT,
        ROBOT_SENSOR_BUFFER_SAMPLE_RING,
        2U,
        10U,
        50U,
        {
            sample_init,
            sample_start,
            sample_stop,
            virtual_configure,
            sample_poll,
            virtual_get_info
        },
        &sample_context
    }
};


bool RobotSensorCatalog_Init(RobotSensorRegistry *registry)
{
    return RobotSensorRegistry_Init(
        registry,
        sensor_catalog,
        (uint8_t)(
            sizeof(sensor_catalog) /
            sizeof(sensor_catalog[0])
        )
    );
}
