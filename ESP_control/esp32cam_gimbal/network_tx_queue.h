#pragma once

#include <stddef.h>
#include <stdint.h>


enum class NetworkTxClass : uint8_t
{
    NORMAL = 0
};


using NetworkTxSendCallback = bool (*)(
    const uint8_t *data,
    size_t length
);

using NetworkTxReadyCallback = bool (*)();


struct NetworkTxStats
{
    uint32_t enqueued;
    uint32_t sent;
    uint32_t droppedFull;
    uint32_t droppedOffline;
    uint32_t sendFailures;
    uint32_t rejected;
};


/*
 * Stage 1 intentionally uses one bounded FIFO. QoS priority queues are added
 * only when the V1 wire semantics are extended in Stage 2.
 */
static constexpr size_t NETWORK_TX_SLOT_COUNT = 3U;


void networkTxInit(
    NetworkTxSendCallback sendCallback,
    NetworkTxReadyCallback readyCallback
);


/*
 * Copy one complete V1 message into queue-owned storage.
 *
 * Returning true means the caller may immediately release or reuse its input
 * buffer. Returning false means no ownership was taken.
 */
bool networkTxEnqueue(
    const uint8_t *data,
    size_t length,
    NetworkTxClass messageClass =
        NetworkTxClass::NORMAL
);


/*
 * Send at most one queued message. Call from the Arduino loop so all network
 * sends have one owner and no callback stack buffer escapes its lifetime.
 */
void networkTxPoll();


size_t networkTxPendingCount();


NetworkTxStats networkTxGetStats();
