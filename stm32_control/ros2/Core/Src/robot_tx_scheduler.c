#include "robot_tx_scheduler.h"

#include <string.h>


static bool deadline_reached(
    uint32_t now_ms,
    uint32_t deadline_ms,
    bool has_deadline
)
{
    return (
        has_deadline &&
        (int32_t)(now_ms - deadline_ms) >= 0
    );
}


#define DEFINE_SLOT_HELPERS(prefix, type, capacity, max_bytes, field) \
static type *prefix##_free(RobotTxScheduler *scheduler) \
{ \
    uint16_t index; \
    for (index = 0U; index < (capacity); ++index) \
    { \
        if (!scheduler->field[index].occupied) \
        { \
            return &scheduler->field[index]; \
        } \
    } \
    return NULL; \
} \
static bool prefix##_store( \
    RobotTxScheduler *scheduler, \
    type *slot, \
    const RobotMessagePolicy *policy, \
    uint32_t key, \
    const uint8_t *data, \
    uint16_t length, \
    uint32_t now_ms) \
{ \
    if (length > (max_bytes)) \
    { \
        scheduler->stats.rejected_oversize++; \
        return false; \
    } \
    memcpy(slot->bytes, data, length); \
    slot->length = length; \
    slot->deadline_ms = policy->deadline_ms == 0U \
        ? 0U : now_ms + policy->deadline_ms; \
    slot->has_deadline = policy->deadline_ms != 0U; \
    slot->order = scheduler->next_order++; \
    slot->key = key; \
    slot->priority = policy->priority; \
    slot->occupied = true; \
    return true; \
}


DEFINE_SLOT_HELPERS(
    reliable,
    RobotTxReliableSlot,
    ROBOT_TX_RELIABLE_CAPACITY,
    ROBOT_TX_RELIABLE_MAX_BYTES,
    reliable
)
DEFINE_SLOT_HELPERS(
    latest,
    RobotTxLatestSlot,
    ROBOT_TX_LATEST_CAPACITY,
    ROBOT_TX_LATEST_MAX_BYTES,
    latest
)
DEFINE_SLOT_HELPERS(
    sample,
    RobotTxSampleSlot,
    ROBOT_TX_SAMPLE_CAPACITY,
    ROBOT_TX_SAMPLE_MAX_BYTES,
    samples
)
DEFINE_SLOT_HELPERS(
    bulk,
    RobotTxBulkSlot,
    ROBOT_TX_BULK_CAPACITY,
    ROBOT_TX_BULK_MAX_BYTES,
    bulk
)


static RobotTxLatestSlot *latest_find_key(
    RobotTxScheduler *scheduler,
    uint32_t key
)
{
    uint16_t index;
    for (index = 0U; index < ROBOT_TX_LATEST_CAPACITY; ++index)
    {
        RobotTxLatestSlot *slot = &scheduler->latest[index];
        if (slot->occupied && slot->key == key)
        {
            return slot;
        }
    }
    return NULL;
}


static RobotTxLatestSlot *latest_oldest(
    RobotTxScheduler *scheduler
)
{
    uint16_t index;
    RobotTxLatestSlot *oldest = NULL;
    for (index = 0U; index < ROBOT_TX_LATEST_CAPACITY; ++index)
    {
        RobotTxLatestSlot *candidate = &scheduler->latest[index];
        if (
            candidate->occupied &&
            (oldest == NULL || candidate->order < oldest->order)
        )
        {
            oldest = candidate;
        }
    }
    return oldest;
}


static RobotTxSampleSlot *sample_oldest(
    RobotTxScheduler *scheduler
)
{
    uint16_t index;
    RobotTxSampleSlot *oldest = NULL;
    for (index = 0U; index < ROBOT_TX_SAMPLE_CAPACITY; ++index)
    {
        RobotTxSampleSlot *candidate = &scheduler->samples[index];
        if (
            candidate->occupied &&
            (oldest == NULL || candidate->order < oldest->order)
        )
        {
            oldest = candidate;
        }
    }
    return oldest;
}


void RobotTxScheduler_Init(RobotTxScheduler *scheduler)
{
    if (scheduler == NULL)
    {
        return;
    }
    memset(scheduler, 0, sizeof(*scheduler));
    scheduler->next_order = 1U;
}


bool RobotTxScheduler_Enqueue(
    RobotTxScheduler *scheduler,
    RobotTxStorageClass storage,
    const RobotMessagePolicy *policy,
    uint32_t key,
    const uint8_t *data,
    uint16_t length,
    uint32_t now_ms
)
{
    bool stored = false;

    if (
        scheduler == NULL ||
        policy == NULL ||
        data == NULL ||
        length == 0U
    )
    {
        return false;
    }

    switch (storage)
    {
        case ROBOT_TX_RELIABLE:
        {
            RobotTxReliableSlot *slot = reliable_free(scheduler);
            if (slot == NULL)
            {
                scheduler->stats.dropped_full++;
                return false;
            }
            stored = reliable_store(
                scheduler, slot, policy, key, data, length, now_ms
            );
            break;
        }
        case ROBOT_TX_BEST_EFFORT_LATEST:
        {
            RobotTxLatestSlot *slot = latest_find_key(scheduler, key);
            if (slot != NULL)
            {
                scheduler->stats.replaced_latest++;
            }
            else
            {
                slot = latest_free(scheduler);
            }
            if (slot == NULL)
            {
                slot = latest_oldest(scheduler);
                scheduler->stats.replaced_latest++;
            }
            stored = latest_store(
                scheduler, slot, policy, key, data, length, now_ms
            );
            break;
        }
        case ROBOT_TX_BEST_EFFORT_SAMPLE:
        {
            RobotTxSampleSlot *slot = sample_free(scheduler);
            if (slot == NULL)
            {
                slot = sample_oldest(scheduler);
                scheduler->stats.dropped_sample++;
            }
            stored = sample_store(
                scheduler, slot, policy, key, data, length, now_ms
            );
            break;
        }
        case ROBOT_TX_BULK:
        {
            RobotTxBulkSlot *slot = bulk_free(scheduler);
            if (slot == NULL)
            {
                scheduler->stats.dropped_full++;
                return false;
            }
            stored = bulk_store(
                scheduler, slot, policy, key, data, length, now_ms
            );
            break;
        }
        default:
            return false;
    }

    if (stored)
    {
        scheduler->stats.enqueued++;
    }
    return stored;
}


#define CONSIDER_SLOTS(field, capacity, prioritize) \
    do \
    { \
        uint16_t candidate_index; \
        for (candidate_index = 0U; \
             candidate_index < (capacity); \
             ++candidate_index) \
        { \
            typeof(scheduler->field[0]) *candidate = \
                &scheduler->field[candidate_index]; \
            if (!candidate->occupied) \
            { \
                continue; \
            } \
            if (deadline_reached( \
                now_ms, \
                candidate->deadline_ms, \
                candidate->has_deadline)) \
            { \
                candidate->occupied = false; \
                scheduler->stats.expired++; \
                continue; \
            } \
            if (selected_bytes == NULL || \
                ((prioritize) && candidate->priority > selected_priority) || \
                (candidate->priority == selected_priority && \
                 candidate->order < selected_order)) \
            { \
                selected_bytes = candidate->bytes; \
                selected_length = candidate->length; \
                selected_order = candidate->order; \
                selected_priority = candidate->priority; \
                selected_occupied = &candidate->occupied; \
            } \
        } \
    } while (0)


bool RobotTxScheduler_TakeNext(
    RobotTxScheduler *scheduler,
    uint32_t now_ms,
    uint8_t *output,
    uint16_t output_capacity,
    uint16_t *output_length
)
{
    const uint8_t *selected_bytes = NULL;
    uint16_t selected_length = 0U;
    uint32_t selected_order = 0U;
    RobotPriority selected_priority = ROBOT_PRIORITY_BULK;
    bool *selected_occupied = NULL;

    if (output_length != NULL)
    {
        *output_length = 0U;
    }
    if (
        scheduler == NULL ||
        output == NULL ||
        output_length == NULL
    )
    {
        return false;
    }

    CONSIDER_SLOTS(reliable, ROBOT_TX_RELIABLE_CAPACITY, true);
    if (selected_bytes == NULL)
    {
        CONSIDER_SLOTS(latest, ROBOT_TX_LATEST_CAPACITY, true);
    }
    if (selected_bytes == NULL)
    {
        CONSIDER_SLOTS(samples, ROBOT_TX_SAMPLE_CAPACITY, false);
    }
    if (selected_bytes == NULL)
    {
        CONSIDER_SLOTS(bulk, ROBOT_TX_BULK_CAPACITY, false);
    }
    if (
        selected_bytes == NULL ||
        selected_occupied == NULL ||
        output_capacity < selected_length
    )
    {
        return false;
    }

    memcpy(output, selected_bytes, selected_length);
    *selected_occupied = false;
    *output_length = selected_length;
    scheduler->stats.dequeued++;
    return true;
}


uint16_t RobotTxScheduler_Pending(
    const RobotTxScheduler *scheduler
)
{
    uint16_t count = 0U;
    uint16_t index;
    if (scheduler == NULL)
    {
        return 0U;
    }
    for (index = 0U; index < ROBOT_TX_RELIABLE_CAPACITY; ++index)
    {
        count += scheduler->reliable[index].occupied ? 1U : 0U;
    }
    for (index = 0U; index < ROBOT_TX_LATEST_CAPACITY; ++index)
    {
        count += scheduler->latest[index].occupied ? 1U : 0U;
    }
    for (index = 0U; index < ROBOT_TX_SAMPLE_CAPACITY; ++index)
    {
        count += scheduler->samples[index].occupied ? 1U : 0U;
    }
    for (index = 0U; index < ROBOT_TX_BULK_CAPACITY; ++index)
    {
        count += scheduler->bulk[index].occupied ? 1U : 0U;
    }
    return count;
}
