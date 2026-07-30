#include <assert.h>
#include <stddef.h>
#include <stdint.h>

#include <iostream>
#include <vector>

#include "network_tx_queue.h"
#include "protocol_v1.h"


namespace
{

bool ready = true;
bool failSend = false;
uint32_t nowMs = 0U;
std::vector<std::vector<uint8_t>> sentMessages;


bool transportReady()
{
    return ready;
}


uint32_t clockNow()
{
    return nowMs;
}


bool captureSend(const uint8_t *data, size_t length)
{
    if (failSend)
    {
        return false;
    }
    sentMessages.emplace_back(data, data + length);
    return true;
}


std::vector<uint8_t> makeMessage(
    uint8_t service,
    uint8_t opcode,
    uint16_t sequence
)
{
    ProtocolV1::MessageView message = {};
    message.version = ProtocolV1::VERSION;
    message.src = ProtocolV1::NODE_STM32;
    message.dst = ProtocolV1::NODE_LINUX;
    message.service = service;
    message.opcode = opcode;
    message.seq = sequence;

    std::vector<uint8_t> result(
        ProtocolV1::MAX_MESSAGE_SIZE
    );
    size_t length = 0U;
    assert(
        ProtocolV1::encodeMessage(
            message,
            result.data(),
            result.size(),
            length
        )
    );
    result.resize(length);
    return result;
}


void resetHarness()
{
    ready = true;
    failSend = false;
    nowMs = 0U;
    sentMessages.clear();
    networkTxInit(captureSend, transportReady, clockNow);
}


void testBulkBacklogNeverDelaysStop()
{
    resetHarness();
    const auto bulk1 = makeMessage(
        ProtocolV1::SERVICE_OTA,
        ProtocolV1::OTA_CHUNK,
        1U
    );
    const auto bulk2 = makeMessage(
        ProtocolV1::SERVICE_OTA,
        ProtocolV1::OTA_CHUNK,
        2U
    );
    const auto bulk3 = makeMessage(
        ProtocolV1::SERVICE_OTA,
        ProtocolV1::OTA_CHUNK,
        3U
    );
    const auto stop = makeMessage(
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_STOP,
        4U
    );

    assert(networkTxEnqueue(bulk1.data(), bulk1.size()));
    assert(networkTxEnqueue(bulk2.data(), bulk2.size()));
    assert(!networkTxEnqueue(bulk3.data(), bulk3.size()));
    assert(networkTxEnqueue(stop.data(), stop.size()));

    networkTxPoll();
    assert(sentMessages.size() == 1U);
    assert(sentMessages[0][4] == ProtocolV1::SERVICE_MOTION);
    assert(sentMessages[0][5] == ProtocolV1::MOTION_STOP);
}


void testLatestReplacementAndSampleRing()
{
    resetHarness();
    for (uint16_t sequence = 0U; sequence < 6U; ++sequence)
    {
        const auto latest = makeMessage(
            ProtocolV1::SERVICE_TELEMETRY,
            ProtocolV1::TELEMETRY_DATA,
            sequence
        );
        assert(networkTxEnqueue(latest.data(), latest.size()));
    }

    assert(networkTxPendingCount() == 1U);
    NetworkTxStats stats = networkTxGetStats();
    assert(stats.replacedLatest == 5U);

    for (
        uint16_t sequence = 0U;
        sequence < NETWORK_TX_SAMPLE_SLOTS + 2U;
        ++sequence
    )
    {
        const auto sample = makeMessage(
            ProtocolV1::SERVICE_TELEMETRY,
            ProtocolV1::TELEMETRY_DATA,
            sequence
        );
        assert(
            networkTxEnqueue(
                sample.data(),
                sample.size(),
                NetworkTxClass::BEST_EFFORT_SAMPLE
            )
        );
    }

    stats = networkTxGetStats();
    assert(stats.droppedSample == 2U);
}


void testExpiredMessageIsNotSent()
{
    resetHarness();
    const auto telemetry = makeMessage(
        ProtocolV1::SERVICE_TELEMETRY,
        ProtocolV1::TELEMETRY_DATA,
        7U
    );
    assert(
        networkTxEnqueue(
            telemetry.data(),
            telemetry.size()
        )
    );

    nowMs = 501U;
    networkTxPoll();

    assert(sentMessages.empty());
    assert(networkTxPendingCount() == 0U);
    assert(networkTxGetStats().expired == 1U);
}


void testOfflineAndSendFailureReleaseSlots()
{
    resetHarness();
    const auto stop = makeMessage(
        ProtocolV1::SERVICE_MOTION,
        ProtocolV1::MOTION_STOP,
        8U
    );
    assert(networkTxEnqueue(stop.data(), stop.size()));
    ready = false;
    networkTxPoll();
    assert(networkTxPendingCount() == 0U);
    assert(networkTxGetStats().droppedOffline == 1U);

    ready = true;
    failSend = true;
    assert(networkTxEnqueue(stop.data(), stop.size()));
    networkTxPoll();
    assert(networkTxGetStats().sendFailures == 1U);
}


void testInvalidInputs()
{
    resetHarness();
    uint8_t invalid[] = {0x01U};
    assert(!networkTxEnqueue(nullptr, 1U));
    assert(!networkTxEnqueue(invalid, 0U));
    assert(!networkTxEnqueue(invalid, sizeof(invalid)));
    assert(networkTxGetStats().rejected == 3U);
}

}


int main()
{
    testBulkBacklogNeverDelaysStop();
    testLatestReplacementAndSampleRing();
    testExpiredMessageIsNotSent();
    testOfflineAndSendFailureReleaseSlots();
    testInvalidInputs();
    std::cout << "Network TX scheduler host tests passed\n";
    return 0;
}
