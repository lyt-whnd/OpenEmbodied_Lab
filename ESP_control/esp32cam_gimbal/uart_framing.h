#pragma once

#include <stddef.h>
#include <stdint.h>

#include "transport_limits.h"


namespace UartFraming
{

static constexpr size_t CRC_SIZE = 2;
static constexpr size_t MAX_APPLICATION_FRAME_SIZE =
    TransportLimits::MAX_OPAQUE_FRAME_SIZE;
static constexpr size_t MAX_RAW_FRAME_SIZE =
    MAX_APPLICATION_FRAME_SIZE + CRC_SIZE;
static constexpr size_t MAX_COBS_FRAME_SIZE =
    MAX_RAW_FRAME_SIZE +
    (MAX_RAW_FRAME_SIZE / 254U) +
    1U;
static constexpr size_t MAX_WIRE_FRAME_SIZE =
    MAX_COBS_FRAME_SIZE + 1U;

enum class DecodeStatus : uint8_t
{
    OK = 0,
    NULL_ARGUMENT,
    INPUT_TOO_LARGE,
    OUTPUT_TOO_SMALL,
    COBS_ERROR,
    FRAME_TOO_SHORT,
    CRC_MISMATCH
};

uint16_t crc16CcittFalse(
    const uint8_t *data,
    size_t length
);

bool cobsEncode(
    const uint8_t *input,
    size_t inputLength,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
);

bool cobsDecode(
    const uint8_t *input,
    size_t inputLength,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
);

bool encodeApplicationFrame(
    const uint8_t *message,
    size_t messageLength,
    uint8_t *wireFrame,
    size_t wireCapacity,
    size_t &wireLength
);

DecodeStatus decodeApplicationFrame(
    const uint8_t *encodedFrame,
    size_t encodedLength,
    uint8_t *message,
    size_t messageCapacity,
    size_t &messageLength
);

const char *decodeStatusName(
    DecodeStatus status
);

}
