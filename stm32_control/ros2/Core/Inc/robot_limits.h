#ifndef ROBOT_LIMITS_H
#define ROBOT_LIMITS_H


/*
 * All communication storage is compile-time bounded for STM32F103C8T6.
 * The RX ring capacity must remain a power of two.
 */
#define ROBOT_UART_RX_RING_CAPACITY       256U
#define ROBOT_TRANSPORT_MAX_POLL_BYTES    256U
#define ROBOT_UART_TX_TIMEOUT_MS          100U
#define ROBOT_RELIABLE_PENDING_CAPACITY   4U
#define ROBOT_RESULT_CACHE_CAPACITY       8U
#define ROBOT_RESULT_CACHE_TTL_MS         30000U

/* Stage-2 QoS scheduler: every byte pool and slot count is fixed. */
#define ROBOT_TX_RELIABLE_CAPACITY         4U
#define ROBOT_TX_RELIABLE_MAX_BYTES        64U
#define ROBOT_TX_LATEST_CAPACITY           2U
#define ROBOT_TX_LATEST_MAX_BYTES          266U
#define ROBOT_TX_SAMPLE_CAPACITY           8U
#define ROBOT_TX_SAMPLE_MAX_BYTES          32U
#define ROBOT_TX_BULK_CAPACITY             2U
#define ROBOT_TX_BULK_MAX_BYTES            266U
#define ROBOT_SENSOR_MAX_REGISTERED         8U
#define ROBOT_SENSOR_MAX_SAMPLE_BYTES       32U
#define ROBOT_TELEMETRY_BATCH_MAX_BYTES     256U
#define ROBOT_TELEMETRY_LATEST_CAPACITY     6U
#define ROBOT_TELEMETRY_FLUSH_PERIOD_MS     50U

/*
 * ceil(100 Hz * 50 ms / 1000) + 3 safety samples = 8.
 * Adjust the rate/window together when a real high-rate source is registered.
 */
#define ROBOT_SAMPLE_TARGET_RATE_HZ         100U
#define ROBOT_SAMPLE_MAX_BUFFER_MS          50U
#define ROBOT_SAMPLE_SAFETY_MARGIN          3U
#define ROBOT_SAMPLE_RING_CAPACITY          \
    (((ROBOT_SAMPLE_TARGET_RATE_HZ * \
       ROBOT_SAMPLE_MAX_BUFFER_MS) + 999U) / 1000U + \
     ROBOT_SAMPLE_SAFETY_MARGIN)
#define ROBOT_SAMPLE_STREAM_CAPACITY        2U
#define ROBOT_SAMPLE_MAX_BYTES              16U


#endif
