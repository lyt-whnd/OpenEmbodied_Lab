#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <iostream>
#include <vector>

#include "network_tx_queue.h"
#include "protocol_v1.h"


namespace
{

bool ready = true;
bool failSend = false;
std::vector<std::vector<uint8_t>> sentMessages;


bool transportReady()
{
    return ready;
}


bool captureSend(
    const uint8_t *data,
    size_t length
)
{
    if (failSend)
    {
        return false;
    }

    sentMessages.emplace_back(
        data,
        data + length
    );
    return true;
}


void resetHarness()
{
    ready = true;
    failSend = false;
    sentMessages.clear();

    networkTxInit(
        captureSend,
        transportReady
    );
}


void pollUntilEmpty()
{
    while (networkTxPendingCount() > 0U)
    {
        networkTxPoll();
    }
}


void testCopyOwnershipFifoAndFullPolicy()
{
    resetHarness();

    uint8_t first[] = {0x01U, 0x02U};
    const uint8_t second[] = {0x03U};
    const uint8_t third[] = {0x04U, 0x05U};
    const uint8_t fourth[] = {0x06U};

    assert(
        networkTxEnqueue(first, sizeof(first))
    );
    assert(
        networkTxEnqueue(second, sizeof(second))
    );
    assert(
        networkTxEnqueue(third, sizeof(third))
    );
    assert(
        !networkTxEnqueue(fourth, sizeof(fourth))
    );

    first[0] = 0xEEU;
    pollUntilEmpty();

    assert(sentMessages.size() == 3U);
    assert(sentMessages[0][0] == 0x01U);
    assert(sentMessages[1][0] == 0x03U);
    assert(sentMessages[2][0] == 0x04U);

    const NetworkTxStats stats =
        networkTxGetStats();

    assert(stats.enqueued == 3U);
    assert(stats.sent == 3U);
    assert(stats.droppedFull == 1U);
    assert(stats.droppedOffline == 0U);
}


void testRapidPongAndStm32ForwardCopies()
{
    resetHarness();

    uint8_t pong[] = {
        0x01U, 0x02U, 0x02U, 0x01U,
        0x01U, 0x02U, 0x09U, 0x00U,
        0x00U, 0x00U
    };
    uint8_t forwarded[] = {
        0x01U, 0x08U, 0x03U, 0x01U,
        0x20U, 0x01U, 0x0AU, 0x00U,
        0x01U, 0x00U, 0x5AU
    };

    assert(
        networkTxEnqueue(
            pong,
            sizeof(pong)
        )
    );
    assert(
        networkTxEnqueue(
            forwarded,
            sizeof(forwarded)
        )
    );

    memset(pong, 0xCC, sizeof(pong));
    memset(forwarded, 0xDD, sizeof(forwarded));

    pollUntilEmpty();

    assert(sentMessages.size() == 2U);
    assert(sentMessages[0][0] == 0x01U);
    assert(sentMessages[0][5] == 0x02U);
    assert(sentMessages[1][2] == 0x03U);
    assert(sentMessages[1][10] == 0x5AU);
}


void testOfflineAndSendFailureReleaseSlots()
{
    resetHarness();

    const uint8_t message[] = {0x11U};

    assert(
        networkTxEnqueue(
            message,
            sizeof(message)
        )
    );
    assert(
        networkTxEnqueue(
            message,
            sizeof(message)
        )
    );

    ready = false;
    networkTxPoll();

    assert(networkTxPendingCount() == 0U);

    NetworkTxStats stats = networkTxGetStats();
    assert(stats.droppedOffline == 2U);

    ready = true;
    failSend = true;

    assert(
        networkTxEnqueue(
            message,
            sizeof(message)
        )
    );
    networkTxPoll();

    assert(networkTxPendingCount() == 0U);

    stats = networkTxGetStats();
    assert(stats.sendFailures == 1U);
}


void testInvalidInputs()
{
    resetHarness();

    uint8_t oversized[
        ProtocolV1::MAX_MESSAGE_SIZE + 1U
    ] = {};

    assert(!networkTxEnqueue(nullptr, 1U));
    assert(!networkTxEnqueue(oversized, 0U));
    assert(
        !networkTxEnqueue(
            oversized,
            sizeof(oversized)
        )
    );

    const NetworkTxStats stats =
        networkTxGetStats();

    assert(stats.rejected == 3U);
    assert(networkTxPendingCount() == 0U);
}

}


int main()
{
    testCopyOwnershipFifoAndFullPolicy();
    testRapidPongAndStm32ForwardCopies();
    testOfflineAndSendFailureReleaseSlots();
    testInvalidInputs();

    std::cout
        << "Network TX queue host tests passed"
        << std::endl;

    return 0;
}
