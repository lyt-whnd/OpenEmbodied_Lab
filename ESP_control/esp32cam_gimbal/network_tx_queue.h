#pragma once

#include <stddef.h>
#include <stdint.h>


enum class NetworkTxClass : uint8_t
{
    AUTO = 0,
    RELIABLE,
    BEST_EFFORT_LATEST,
    BEST_EFFORT_SAMPLE,
    BULK
};


using NetworkTxSendCallback = bool (*)(
    const uint8_t *data,
    size_t length
);

using NetworkTxReadyCallback = bool (*)();
using NetworkTxClockCallback = uint32_t (*)();


struct NetworkTxStats
{
    uint32_t enqueued;
    uint32_t sent;
    uint32_t droppedFull;
    uint32_t droppedOffline;
    uint32_t replacedLatest;
    uint32_t droppedSample;
    uint32_t expired;
    uint32_t sendFailures;
    uint32_t rejected;
};


static constexpr size_t NETWORK_TX_RELIABLE_SLOTS = 4U;
static constexpr size_t NETWORK_TX_LATEST_SLOTS = 4U;
static constexpr size_t NETWORK_TX_SAMPLE_SLOTS = 8U;
static constexpr size_t NETWORK_TX_BULK_SLOTS = 2U;


void networkTxInit(
    NetworkTxSendCallback sendCallback,
    NetworkTxReadyCallback readyCallback,
    NetworkTxClockCallback clockCallback = nullptr
);


/*
 * Copy one complete V1 message into class-specific bounded storage.
 * AUTO resolves QoS, priority, deadline, and overflow from service/opcode.
 */
bool networkTxEnqueue(
    const uint8_t *data,
    size_t length,
    NetworkTxClass messageClass =
        NetworkTxClass::AUTO
);


/* Send at most one message in RELIABLE/latest/sample/BULK order. */
void networkTxPoll();


size_t networkTxPendingCount();


NetworkTxStats networkTxGetStats();
