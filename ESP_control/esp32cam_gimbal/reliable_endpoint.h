#pragma once

#include <stddef.h>
#include <stdint.h>

#include "protocol_v1.h"
#include "result_cache.h"


namespace Reliable
{

static constexpr uint8_t SCHEMA_VERSION = 1U;
static constexpr size_t REQUEST_HEADER_SIZE = 7U;
static constexpr size_t RESULT_PAYLOAD_SIZE = 10U;

enum class BeginStatus : uint8_t
{
    NEW_REQUEST,
    DUPLICATE,
    INVALID
};

struct RequestView
{
    uint16_t epoch;
    uint32_t requestId;
    const uint8_t *payload;
    uint16_t payloadLength;
};

using SendCallback = bool (*)(
    const uint8_t *data,
    size_t length
);

class Endpoint
{
public:
    Endpoint();

    void reset();

    BeginStatus begin(
        const ProtocolV1::MessageView &message,
        uint32_t nowMs,
        RequestView &request,
        CachedResult &cached
    );

    void complete(
        const ProtocolV1::MessageView &message,
        const RequestView &request,
        ResultStage stage,
        uint16_t status,
        uint32_t nowMs
    );

    bool sendResult(
        const ProtocolV1::MessageView &requestMessage,
        const RequestView &request,
        ResultStage stage,
        uint16_t status,
        SendCallback send
    ) const;

private:
    ResultCache cache_;
};

}
