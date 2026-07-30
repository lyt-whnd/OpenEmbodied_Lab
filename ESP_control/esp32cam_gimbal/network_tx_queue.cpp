#include "network_tx_queue.h"

#include <string.h>

#include "protocol_v1.h"
#include "service_registry.h"


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
    uint8_t data[ProtocolV1::MAX_MESSAGE_SIZE];
    size_t length;
    uint32_t deadlineMs;
    uint32_t order;
    uint32_t key;
    ServiceRegistry::Priority priority;
    bool hasDeadline;
    bool occupied;
};


NetworkTxSlot reliableSlots[
    NETWORK_TX_RELIABLE_SLOTS
] = {};
NetworkTxSlot latestSlots[
    NETWORK_TX_LATEST_SLOTS
] = {};
NetworkTxSlot sampleSlots[
    NETWORK_TX_SAMPLE_SLOTS
] = {};
NetworkTxSlot bulkSlots[
    NETWORK_TX_BULK_SLOTS
] = {};

uint32_t nextOrder = 1U;
NetworkTxSendCallback sendMessage = nullptr;
NetworkTxReadyCallback transportIsReady = nullptr;
NetworkTxClockCallback readClock = nullptr;
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


uint32_t currentTimeMs()
{
    return readClock == nullptr ? 0U : readClock();
}


bool deadlineReached(
    uint32_t nowMs,
    uint32_t deadlineMs,
    bool hasDeadline
)
{
    return (
        hasDeadline &&
        static_cast<int32_t>(nowMs - deadlineMs) >= 0
    );
}


template <size_t Capacity>
size_t occupiedCount(
    const NetworkTxSlot (&slots)[Capacity]
)
{
    size_t count = 0U;
    for (size_t index = 0U; index < Capacity; ++index)
    {
        count += slots[index].occupied ? 1U : 0U;
    }
    return count;
}


template <size_t Capacity>
void clearSlots(NetworkTxSlot (&slots)[Capacity])
{
    for (size_t index = 0U; index < Capacity; ++index)
    {
        slots[index].occupied = false;
    }
}


void clearPendingLocked()
{
    clearSlots(reliableSlots);
    clearSlots(latestSlots);
    clearSlots(sampleSlots);
    clearSlots(bulkSlots);
}


size_t pendingCountLocked()
{
    return (
        occupiedCount(reliableSlots)
        + occupiedCount(latestSlots)
        + occupiedCount(sampleSlots)
        + occupiedCount(bulkSlots)
    );
}


template <size_t Capacity>
NetworkTxSlot *findFree(
    NetworkTxSlot (&slots)[Capacity]
)
{
    for (size_t index = 0U; index < Capacity; ++index)
    {
        if (!slots[index].occupied)
        {
            return &slots[index];
        }
    }
    return nullptr;
}


template <size_t Capacity>
NetworkTxSlot *findOldest(
    NetworkTxSlot (&slots)[Capacity]
)
{
    NetworkTxSlot *oldest = nullptr;
    for (size_t index = 0U; index < Capacity; ++index)
    {
        NetworkTxSlot &candidate = slots[index];
        if (
            candidate.occupied &&
            (
                oldest == nullptr ||
                candidate.order < oldest->order
            )
        )
        {
            oldest = &candidate;
        }
    }
    return oldest;
}


template <size_t Capacity>
NetworkTxSlot *findKey(
    NetworkTxSlot (&slots)[Capacity],
    uint32_t key
)
{
    for (size_t index = 0U; index < Capacity; ++index)
    {
        if (slots[index].occupied && slots[index].key == key)
        {
            return &slots[index];
        }
    }
    return nullptr;
}


void storeSlot(
    NetworkTxSlot &slot,
    const uint8_t *data,
    size_t length,
    const ServiceRegistry::MessagePolicy &policy,
    uint32_t key,
    uint32_t nowMs
)
{
    memcpy(slot.data, data, length);
    slot.length = length;
    slot.deadlineMs = (
        policy.deadlineMs == 0U
        ? 0U
        : nowMs + policy.deadlineMs
    );
    slot.hasDeadline = policy.deadlineMs != 0U;
    slot.order = nextOrder++;
    slot.key = key;
    slot.priority = policy.priority;
    slot.occupied = true;
}


NetworkTxClass classFromPolicy(
    const ServiceRegistry::MessagePolicy &policy
)
{
    switch (policy.qos)
    {
        case ServiceRegistry::QosClass::RELIABLE:
            return NetworkTxClass::RELIABLE;
        case ServiceRegistry::QosClass::BULK:
            return NetworkTxClass::BULK;
        case ServiceRegistry::QosClass::BEST_EFFORT:
        default:
            return NetworkTxClass::BEST_EFFORT_LATEST;
    }
}


template <size_t Capacity>
NetworkTxSlot *takeCandidate(
    NetworkTxSlot (&slots)[Capacity],
    uint32_t nowMs,
    bool prioritize
)
{
    NetworkTxSlot *selected = nullptr;
    for (size_t index = 0U; index < Capacity; ++index)
    {
        NetworkTxSlot &candidate = slots[index];
        if (!candidate.occupied)
        {
            continue;
        }
        if (deadlineReached(
            nowMs,
            candidate.deadlineMs,
            candidate.hasDeadline
        ))
        {
            candidate.occupied = false;
            stats.expired++;
            continue;
        }
        if (
            selected == nullptr ||
            (
                prioritize &&
                candidate.priority > selected->priority
            ) ||
            (
                candidate.priority == selected->priority &&
                candidate.order < selected->order
            )
        )
        {
            selected = &candidate;
        }
    }
    return selected;
}

}


void networkTxInit(
    NetworkTxSendCallback sendCallback,
    NetworkTxReadyCallback readyCallback,
    NetworkTxClockCallback clockCallback
)
{
    lockQueue();
    sendMessage = sendCallback;
    transportIsReady = readyCallback;
    readClock = clockCallback;
    stats = {};
    nextOrder = 1U;
    clearPendingLocked();
    unlockQueue();
}


bool networkTxEnqueue(
    const uint8_t *data,
    size_t length,
    NetworkTxClass messageClass
)
{
    ProtocolV1::MessageView message = {};
    if (
        data == nullptr ||
        length == 0U ||
        length > ProtocolV1::MAX_MESSAGE_SIZE ||
        ProtocolV1::decodeMessage(
            data,
            length,
            message
        ) != ProtocolV1::DecodeStatus::OK
    )
    {
        lockQueue();
        stats.rejected++;
        unlockQueue();
        return false;
    }

    const ServiceRegistry::MessagePolicy *policy =
        ServiceRegistry::lookup(
            message.service,
            message.opcode
        );
    if (policy == nullptr)
    {
        lockQueue();
        stats.rejected++;
        unlockQueue();
        return false;
    }

    if (messageClass == NetworkTxClass::AUTO)
    {
        messageClass = classFromPolicy(*policy);
    }

    const uint32_t key = (
        (static_cast<uint32_t>(message.src) << 24U) |
        (static_cast<uint32_t>(message.dst) << 16U) |
        (static_cast<uint32_t>(message.service) << 8U) |
        static_cast<uint32_t>(message.opcode)
    );
    const uint32_t nowMs = currentTimeMs();

    lockQueue();
    NetworkTxSlot *slot = nullptr;

    switch (messageClass)
    {
        case NetworkTxClass::RELIABLE:
            slot = findFree(reliableSlots);
            break;
        case NetworkTxClass::BEST_EFFORT_LATEST:
            slot = findKey(latestSlots, key);
            if (slot != nullptr)
            {
                stats.replacedLatest++;
            }
            else
            {
                slot = findFree(latestSlots);
            }
            if (slot == nullptr)
            {
                slot = findOldest(latestSlots);
                stats.replacedLatest++;
            }
            break;
        case NetworkTxClass::BEST_EFFORT_SAMPLE:
            slot = findFree(sampleSlots);
            if (slot == nullptr)
            {
                slot = findOldest(sampleSlots);
                stats.droppedSample++;
            }
            break;
        case NetworkTxClass::BULK:
            slot = findFree(bulkSlots);
            break;
        case NetworkTxClass::AUTO:
        default:
            break;
    }

    if (slot == nullptr)
    {
        stats.droppedFull++;
        unlockQueue();
        return false;
    }

    storeSlot(*slot, data, length, *policy, key, nowMs);
    stats.enqueued++;
    unlockQueue();
    return true;
}


void networkTxPoll()
{
    NetworkTxReadyCallback readyCallback = nullptr;
    NetworkTxSendCallback sendCallback = nullptr;

    lockQueue();
    const size_t pending = pendingCountLocked();
    readyCallback = transportIsReady;
    sendCallback = sendMessage;
    unlockQueue();

    if (pending == 0U)
    {
        return;
    }

    if (readyCallback == nullptr || !readyCallback())
    {
        lockQueue();
        stats.droppedOffline +=
            static_cast<uint32_t>(
                occupiedCount(reliableSlots)
                + occupiedCount(latestSlots)
                + occupiedCount(sampleSlots)
                + occupiedCount(bulkSlots)
            );
        clearPendingLocked();
        unlockQueue();
        return;
    }

    uint8_t packet[ProtocolV1::MAX_MESSAGE_SIZE] = {};
    size_t packetLength = 0U;
    const uint32_t nowMs = currentTimeMs();

    lockQueue();
    NetworkTxSlot *slot =
        takeCandidate(reliableSlots, nowMs, true);
    if (slot == nullptr)
    {
        slot = takeCandidate(latestSlots, nowMs, true);
    }
    if (slot == nullptr)
    {
        slot = takeCandidate(sampleSlots, nowMs, false);
    }
    if (slot == nullptr)
    {
        slot = takeCandidate(bulkSlots, nowMs, false);
    }
    if (slot != nullptr)
    {
        packetLength = slot->length;
        memcpy(packet, slot->data, packetLength);
        slot->occupied = false;
    }
    unlockQueue();

    if (packetLength == 0U)
    {
        return;
    }

    const bool sent = (
        sendCallback != nullptr &&
        sendCallback(packet, packetLength)
    );

    lockQueue();
    if (sent)
    {
        stats.sent++;
    }
    else
    {
        stats.sendFailures++;
    }
    unlockQueue();
}


size_t networkTxPendingCount()
{
    lockQueue();
    const size_t result = pendingCountLocked();
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
