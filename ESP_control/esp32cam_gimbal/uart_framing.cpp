#include "uart_framing.h"

#include <string.h>


namespace UartFraming
{

uint16_t crc16CcittFalse(
    const uint8_t *data,
    size_t length
)
{
    uint16_t crc = 0xFFFFU;

    for (size_t index = 0; index < length; index++)
    {
        crc ^= static_cast<uint16_t>(
            static_cast<uint16_t>(data[index])
            << 8U
        );

        for (uint8_t bit = 0; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = static_cast<uint16_t>(
                    (crc << 1U) ^ 0x1021U
                );
            }
            else
            {
                crc = static_cast<uint16_t>(
                    crc << 1U
                );
            }
        }
    }

    return crc;
}


bool cobsEncode(
    const uint8_t *input,
    size_t inputLength,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
)
{
    outputLength = 0;

    if (
        input == nullptr ||
        output == nullptr ||
        outputCapacity == 0U
    )
    {
        return false;
    }

    size_t readIndex = 0;
    size_t writeIndex = 1;
    size_t codeIndex = 0;
    uint8_t code = 1;

    while (readIndex < inputLength)
    {
        if (input[readIndex] == 0U)
        {
            output[codeIndex] = code;
            code = 1;
            codeIndex = writeIndex;

            if (writeIndex >= outputCapacity)
            {
                return false;
            }

            writeIndex++;
            readIndex++;
            continue;
        }

        if (writeIndex >= outputCapacity)
        {
            return false;
        }

        output[writeIndex] = input[readIndex];
        writeIndex++;
        readIndex++;
        code++;

        if (code == 0xFFU)
        {
            output[codeIndex] = code;
            code = 1;
            codeIndex = writeIndex;

            if (writeIndex >= outputCapacity)
            {
                return false;
            }

            writeIndex++;
        }
    }

    output[codeIndex] = code;
    outputLength = writeIndex;

    return true;
}


bool cobsDecode(
    const uint8_t *input,
    size_t inputLength,
    uint8_t *output,
    size_t outputCapacity,
    size_t &outputLength
)
{
    outputLength = 0;

    if (
        input == nullptr ||
        output == nullptr ||
        inputLength == 0U
    )
    {
        return false;
    }

    size_t readIndex = 0;
    size_t writeIndex = 0;

    while (readIndex < inputLength)
    {
        const uint8_t code = input[readIndex];

        if (code == 0U)
        {
            return false;
        }

        readIndex++;

        for (uint8_t offset = 1; offset < code; offset++)
        {
            if (
                readIndex >= inputLength ||
                writeIndex >= outputCapacity
            )
            {
                return false;
            }

            output[writeIndex] = input[readIndex];
            writeIndex++;
            readIndex++;
        }

        if (
            code != 0xFFU &&
            readIndex < inputLength
        )
        {
            if (writeIndex >= outputCapacity)
            {
                return false;
            }

            output[writeIndex] = 0;
            writeIndex++;
        }
    }

    outputLength = writeIndex;

    return true;
}


bool encodeApplicationFrame(
    const uint8_t *message,
    size_t messageLength,
    uint8_t *wireFrame,
    size_t wireCapacity,
    size_t &wireLength
)
{
    wireLength = 0;

    if (
        message == nullptr ||
        wireFrame == nullptr ||
        messageLength > ProtocolV1::MAX_MESSAGE_SIZE ||
        wireCapacity < 2U
    )
    {
        return false;
    }

    uint8_t rawFrame[MAX_RAW_FRAME_SIZE] = {};

    memcpy(
        rawFrame,
        message,
        messageLength
    );

    const uint16_t crc =
        crc16CcittFalse(
            message,
            messageLength
        );

    rawFrame[messageLength] =
        static_cast<uint8_t>(crc & 0xFFU);
    rawFrame[messageLength + 1U] =
        static_cast<uint8_t>(crc >> 8U);

    size_t encodedLength = 0;

    if (
        !cobsEncode(
            rawFrame,
            messageLength + CRC_SIZE,
            wireFrame,
            wireCapacity - 1U,
            encodedLength
        )
    )
    {
        return false;
    }

    wireFrame[encodedLength] = 0;
    wireLength = encodedLength + 1U;

    return true;
}


DecodeStatus decodeApplicationFrame(
    const uint8_t *encodedFrame,
    size_t encodedLength,
    uint8_t *message,
    size_t messageCapacity,
    size_t &messageLength
)
{
    messageLength = 0;

    if (
        encodedFrame == nullptr ||
        message == nullptr
    )
    {
        return DecodeStatus::NULL_ARGUMENT;
    }

    if (encodedLength > MAX_COBS_FRAME_SIZE)
    {
        return DecodeStatus::INPUT_TOO_LARGE;
    }

    uint8_t rawFrame[MAX_RAW_FRAME_SIZE] = {};
    size_t rawLength = 0;

    if (
        !cobsDecode(
            encodedFrame,
            encodedLength,
            rawFrame,
            sizeof(rawFrame),
            rawLength
        )
    )
    {
        return DecodeStatus::COBS_ERROR;
    }

    if (rawLength < CRC_SIZE)
    {
        return DecodeStatus::FRAME_TOO_SHORT;
    }

    const size_t applicationLength =
        rawLength - CRC_SIZE;

    if (applicationLength > messageCapacity)
    {
        return DecodeStatus::OUTPUT_TOO_SMALL;
    }

    const uint16_t receivedCrc =
        static_cast<uint16_t>(
            static_cast<uint16_t>(
                rawFrame[applicationLength]
            ) |
            (
                static_cast<uint16_t>(
                    rawFrame[applicationLength + 1U]
                )
                << 8U
            )
        );

    const uint16_t expectedCrc =
        crc16CcittFalse(
            rawFrame,
            applicationLength
        );

    if (receivedCrc != expectedCrc)
    {
        return DecodeStatus::CRC_MISMATCH;
    }

    memcpy(
        message,
        rawFrame,
        applicationLength
    );

    messageLength = applicationLength;

    return DecodeStatus::OK;
}


const char *decodeStatusName(
    DecodeStatus status
)
{
    switch (status)
    {
        case DecodeStatus::OK:
            return "OK";

        case DecodeStatus::NULL_ARGUMENT:
            return "NULL_ARGUMENT";

        case DecodeStatus::INPUT_TOO_LARGE:
            return "INPUT_TOO_LARGE";

        case DecodeStatus::OUTPUT_TOO_SMALL:
            return "OUTPUT_TOO_SMALL";

        case DecodeStatus::COBS_ERROR:
            return "COBS_ERROR";

        case DecodeStatus::FRAME_TOO_SHORT:
            return "FRAME_TOO_SHORT";

        case DecodeStatus::CRC_MISMATCH:
            return "CRC_MISMATCH";

        default:
            return "UNKNOWN_STATUS";
    }
}

}
