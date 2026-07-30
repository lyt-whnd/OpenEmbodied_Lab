#include "network_tx_queue.h"

#include <string.h>

#include "protocol_v1.h"


#if defined(ARDUINO_ARCH_ESP32)

#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>

#else

#include <mutex>

#endif


namespace
{

struct NetworkTxSlot
{
    uint8_t data[
        ProtocolV1::MAX_MESSAGE_SIZE
    ];
    size_t length;
    NetworkTxClass messageClass;
};


NetworkTxSlot slots[NETWORK_TX_SLOT_COUNT] = {};
size_t headIndex = 0;
size_t tailIndex = 0;
size_t pendingCount = 0;
bool pollInProgress = false;
NetworkTxSendCallback sendMessage = nullptr;
NetworkTxReadyCallback transportIsReady = nullptr;
NetworkTxStats stats = {};


#if defined(ARDUINO_ARCH_ESP32)

portMUX_TYPE queueMutex =
    portMUX_INITIALIZER_UNLOCKED;


void lockQueue()
{
    portENTER_CRITICAL(&queueMutex);
}


void unlockQueue()
{
    portEXIT_CRITICAL(&queueMutex);
}

#else

std::mutex queueMutex;


void lockQueue()
{
    queueMutex.lock();
}


void unlockQueue()
{
    queueMutex.unlock();
}

#endif


void clearPendingLocked()
{
    headIndex = 0;
    tailIndex = 0;
    pendingCount = 0;
}

}


void networkTxInit(
    NetworkTxSendCallback sendCallback,
    NetworkTxReadyCallback readyCallback
)
{
    lockQueue();

    sendMessage = sendCallback;
    transportIsReady = readyCallback;
    pollInProgress = false;
    stats = {};
    clearPendingLocked();

    unlockQueue();
}


bool networkTxEnqueue(
    const uint8_t *data,
    size_t length,
    NetworkTxClass messageClass
)
{
    if (
        data == nullptr ||
        length == 0U ||
        length > ProtocolV1::MAX_MESSAGE_SIZE
    )
    {
        lockQueue();
        stats.rejected++;
        unlockQueue();
        return false;
    }

    lockQueue();

    if (pendingCount >= NETWORK_TX_SLOT_COUNT)
    {
        stats.droppedFull++;
        unlockQueue();
        return false;
    }

    NetworkTxSlot &slot = slots[tailIndex];

    memcpy(
        slot.data,
        data,
        length
    );
    slot.length = length;
    slot.messageClass = messageClass;

    tailIndex =
        (tailIndex + 1U) %
        NETWORK_TX_SLOT_COUNT;
    pendingCount++;
    stats.enqueued++;

    unlockQueue();
    return true;
}


void networkTxPoll()
{
    NetworkTxReadyCallback readyCallback = nullptr;
    NetworkTxSendCallback sendCallback = nullptr;

    lockQueue();

    if (
        pollInProgress ||
        pendingCount == 0U
    )
    {
        unlockQueue();
        return;
    }

    readyCallback = transportIsReady;
    sendCallback = sendMessage;

    unlockQueue();

    const bool ready = (
        readyCallback != nullptr &&
        readyCallback()
    );

    if (!ready)
    {
        lockQueue();
        stats.droppedOffline +=
            static_cast<uint32_t>(pendingCount);
        clearPendingLocked();
        unlockQueue();
        return;
    }

    lockQueue();

    if (
        pollInProgress ||
        pendingCount == 0U
    )
    {
        unlockQueue();
        return;
    }

    pollInProgress = true;
    NetworkTxSlot *slot = &slots[headIndex];

    unlockQueue();

    const bool sent = (
        sendCallback != nullptr &&
        sendCallback(
            slot->data,
            slot->length
        )
    );

    lockQueue();

    headIndex =
        (headIndex + 1U) %
        NETWORK_TX_SLOT_COUNT;
    pendingCount--;

    if (sent)
    {
        stats.sent++;
    }
    else
    {
        stats.sendFailures++;
    }

    pollInProgress = false;

    unlockQueue();
}


size_t networkTxPendingCount()
{
    lockQueue();
    const size_t result = pendingCount;
    unlockQueue();
    return result;
}


NetworkTxStats networkTxGetStats()
{
    lockQueue();
    const NetworkTxStats result = stats;
    unlockQueue();
    return result;
}
