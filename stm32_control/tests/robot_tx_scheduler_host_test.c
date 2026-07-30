#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "robot_tx_scheduler.h"


static RobotMessagePolicy policy(
    RobotQosClass qos,
    RobotPriority priority,
    uint16_t deadline_ms
)
{
    RobotMessagePolicy result = {
        0U,
        0U,
        qos,
        priority,
        deadline_ms,
        ROBOT_OVERFLOW_DROP_OLD
    };
    return result;
}


static void test_bulk_does_not_delay_stop(void)
{
    RobotTxScheduler scheduler;
    uint8_t output[ROBOT_TX_BULK_MAX_BYTES];
    uint16_t length = 0U;
    const uint8_t bulk[] = {0xB0U};
    const uint8_t stop[] = {0x5AU};
    RobotMessagePolicy bulk_policy = policy(
        ROBOT_QOS_BULK, ROBOT_PRIORITY_BULK, 0U
    );
    RobotMessagePolicy stop_policy = policy(
        ROBOT_QOS_RELIABLE, ROBOT_PRIORITY_EMERGENCY, 250U
    );

    RobotTxScheduler_Init(&scheduler);
    assert(RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_BULK, &bulk_policy,
        1U, bulk, sizeof(bulk), 0U
    ));
    assert(RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_BULK, &bulk_policy,
        2U, bulk, sizeof(bulk), 0U
    ));
    assert(!RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_BULK, &bulk_policy,
        3U, bulk, sizeof(bulk), 0U
    ));
    assert(RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_RELIABLE, &stop_policy,
        4U, stop, sizeof(stop), 0U
    ));

    assert(RobotTxScheduler_TakeNext(
        &scheduler, 1U, output, sizeof(output), &length
    ));
    assert(length == 1U && output[0] == 0x5AU);
    assert(scheduler.stats.dropped_full == 1U);
}


static void test_latest_sample_and_expiry_counters(void)
{
    RobotTxScheduler scheduler;
    uint8_t output[ROBOT_TX_LATEST_MAX_BYTES];
    uint16_t length = 0U;
    uint8_t value;
    uint16_t index;
    RobotMessagePolicy latest_policy = policy(
        ROBOT_QOS_BEST_EFFORT, ROBOT_PRIORITY_HIGH, 10U
    );
    RobotMessagePolicy sample_policy = policy(
        ROBOT_QOS_BEST_EFFORT, ROBOT_PRIORITY_NORMAL, 0U
    );

    RobotTxScheduler_Init(&scheduler);
    value = 1U;
    assert(RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_BEST_EFFORT_LATEST,
        &latest_policy, 7U, &value, 1U, 0U
    ));
    value = 2U;
    assert(RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_BEST_EFFORT_LATEST,
        &latest_policy, 7U, &value, 1U, 0U
    ));
    assert(RobotTxScheduler_Pending(&scheduler) == 1U);
    assert(scheduler.stats.replaced_latest == 1U);

    assert(!RobotTxScheduler_TakeNext(
        &scheduler, 11U, output, sizeof(output), &length
    ));
    assert(scheduler.stats.expired == 1U);

    for (
        index = 0U;
        index < ROBOT_TX_SAMPLE_CAPACITY + 1U;
        ++index
    )
    {
        value = (uint8_t)index;
        assert(RobotTxScheduler_Enqueue(
            &scheduler, ROBOT_TX_BEST_EFFORT_SAMPLE,
            &sample_policy, 9U, &value, 1U, 20U
        ));
    }
    assert(scheduler.stats.dropped_sample == 1U);
    assert(RobotTxScheduler_TakeNext(
        &scheduler, 20U, output, sizeof(output), &length
    ));
    assert(output[0] == 1U);
}


static void test_class_specific_size_limit(void)
{
    RobotTxScheduler scheduler;
    uint8_t oversized[ROBOT_TX_RELIABLE_MAX_BYTES + 1U] = {0};
    RobotMessagePolicy reliable_policy = policy(
        ROBOT_QOS_RELIABLE, ROBOT_PRIORITY_HIGH, 100U
    );

    RobotTxScheduler_Init(&scheduler);
    assert(!RobotTxScheduler_Enqueue(
        &scheduler, ROBOT_TX_RELIABLE, &reliable_policy,
        1U, oversized, sizeof(oversized), 0U
    ));
    assert(scheduler.stats.rejected_oversize == 1U);
}


int main(void)
{
    test_bulk_does_not_delay_stop();
    test_latest_sample_and_expiry_counters();
    test_class_specific_size_limit();
    puts("STM32 TX scheduler host tests passed");
    return 0;
}
