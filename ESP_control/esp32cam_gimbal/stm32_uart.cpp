#include "stm32_uart.h"

#include <Arduino.h>

#include "app_config.h"
#include "protocol_v1.h"
#include "uart_framing.h"


namespace
{

/*
 * UART0 被下载底座和 Serial 调试占用。
 *
 * 这里使用 UART1 与 STM32 通信，
 * 并将 UART1 映射到 GPIO13 和 GPIO14。
 */
HardwareSerial stm32Serial(1);

bool uartInitialized = false;
bool discardUntilDelimiter = false;

Stm32UartMessageCallback messageCallback =
    nullptr;

uint8_t encodedReceiveBuffer[
    UartFraming::MAX_COBS_FRAME_SIZE
];

size_t encodedReceiveLength = 0;


void processEncodedFrame()
{
    uint8_t message[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};

    size_t messageLength = 0;

    const UartFraming::DecodeStatus frameStatus =
        UartFraming::decodeApplicationFrame(
            encodedReceiveBuffer,
            encodedReceiveLength,
            message,
            sizeof(message),
            messageLength
        );

    if (frameStatus != UartFraming::DecodeStatus::OK)
    {
        Serial.printf(
            "STM32 RX frame rejected: %s\n",
            UartFraming::decodeStatusName(
                frameStatus
            )
        );

        return;
    }

    ProtocolV1::MessageView view = {};

    const ProtocolV1::DecodeStatus messageStatus =
        ProtocolV1::decodeMessage(
            message,
            messageLength,
            view
        );

    if (
        messageStatus !=
        ProtocolV1::DecodeStatus::OK
    )
    {
        Serial.printf(
            "STM32 RX V1 message rejected: %s\n",
            ProtocolV1::decodeStatusName(
                messageStatus
            )
        );

        return;
    }

    if (view.src != ProtocolV1::NODE_STM32)
    {
        Serial.printf(
            "STM32 RX rejected: invalid src=0x%02X\n",
            static_cast<unsigned int>(view.src)
        );

        return;
    }

    if (
        view.dst != ProtocolV1::NODE_LINUX &&
        view.dst != ProtocolV1::NODE_ESP32 &&
        view.dst != ProtocolV1::NODE_BROADCAST
    )
    {
        Serial.printf(
            "STM32 RX rejected: invalid dst=0x%02X\n",
            static_cast<unsigned int>(view.dst)
        );

        return;
    }

    Serial.printf(
        "STM32 RX V1: seq=%u, "
        "service=0x%02X, opcode=0x%02X, "
        "payload=%u\n",
        static_cast<unsigned int>(view.seq),
        static_cast<unsigned int>(view.service),
        static_cast<unsigned int>(view.opcode),
        static_cast<unsigned int>(
            view.payloadLength
        )
    );

    if (messageCallback != nullptr)
    {
        messageCallback(
            message,
            messageLength
        );
    }
}

}


bool stm32UartInit()
{
    stm32Serial.begin(
        AppConfig::Stm32Uart::BAUD_RATE,
        SERIAL_8N1,
        AppConfig::Stm32Uart::RX_PIN,
        AppConfig::Stm32Uart::TX_PIN
    );

    encodedReceiveLength = 0;
    discardUntilDelimiter = false;
    uartInitialized = true;

    Serial.println();
    Serial.println("STM32 V1 UART initialized");

    Serial.printf(
        "STM32 UART: baud=%u, RX=%d, TX=%d\n",
        static_cast<unsigned int>(
            AppConfig::Stm32Uart::BAUD_RATE
        ),
        AppConfig::Stm32Uart::RX_PIN,
        AppConfig::Stm32Uart::TX_PIN
    );

    return true;
}


void stm32UartSetMessageCallback(
    Stm32UartMessageCallback callback
)
{
    messageCallback = callback;
}


bool stm32UartSendApplicationMessage(
    const uint8_t *message,
    size_t length
)
{
    if (!uartInitialized)
    {
        Serial.println(
            "STM32 UART is not initialized"
        );

        return false;
    }

    ProtocolV1::MessageView view = {};

    const ProtocolV1::DecodeStatus status =
        ProtocolV1::decodeMessage(
            message,
            length,
            view
        );

    if (status != ProtocolV1::DecodeStatus::OK)
    {
        Serial.printf(
            "STM32 TX rejected invalid V1 message: %s\n",
            ProtocolV1::decodeStatusName(status)
        );

        return false;
    }

    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};

    size_t wireLength = 0;

    if (
        !UartFraming::encodeApplicationFrame(
            message,
            length,
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    )
    {
        Serial.println(
            "STM32 TX frame encoding failed"
        );

        return false;
    }

    const size_t bytesWritten =
        stm32Serial.write(
            wireFrame,
            wireLength
        );

    if (bytesWritten != wireLength)
    {
        Serial.printf(
            "STM32 TX incomplete: %u/%u bytes\n",
            static_cast<unsigned int>(
                bytesWritten
            ),
            static_cast<unsigned int>(
                wireLength
            )
        );

        return false;
    }

    Serial.printf(
        "STM32 TX V1: seq=%u, "
        "service=0x%02X, opcode=0x%02X, "
        "wire=%u bytes\n",
        static_cast<unsigned int>(view.seq),
        static_cast<unsigned int>(view.service),
        static_cast<unsigned int>(view.opcode),
        static_cast<unsigned int>(wireLength)
    );

    return true;
}


void stm32UartPoll()
{
    if (!uartInitialized)
    {
        return;
    }

    while (stm32Serial.available() > 0)
    {
        const uint8_t value =
            static_cast<uint8_t>(
                stm32Serial.read()
            );

        if (value == 0U)
        {
            if (discardUntilDelimiter)
            {
                discardUntilDelimiter = false;
                encodedReceiveLength = 0;
                continue;
            }

            if (encodedReceiveLength > 0U)
            {
                processEncodedFrame();
                encodedReceiveLength = 0;
            }

            continue;
        }

        if (discardUntilDelimiter)
        {
            continue;
        }

        if (
            encodedReceiveLength >=
            sizeof(encodedReceiveBuffer)
        )
        {
            Serial.println(
                "STM32 RX frame too long, discarded"
            );

            encodedReceiveLength = 0;
            discardUntilDelimiter = true;
            continue;
        }

        encodedReceiveBuffer[
            encodedReceiveLength
        ] = value;

        encodedReceiveLength++;
    }
}
