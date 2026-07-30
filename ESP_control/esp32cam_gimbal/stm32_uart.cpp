#include "stm32_uart.h"

#include <Arduino.h>

#include "app_config.h"
#include "uart_framing.h"
#include "uart_stream_framer.h"


namespace
{

/*
 * UART0 is occupied by flashing and Serial diagnostics. UART1 is mapped to
 * GPIO13/GPIO14 for the STM32 link.
 */
HardwareSerial stm32Serial(1);

bool transportInitialized = false;
Stm32TransportReceiveCallback receiveCallback =
    nullptr;
UartStreamFramer::Receiver streamReceiver;


void processEncodedFrame(
    const uint8_t *encodedData,
    size_t encodedLength
)
{
    uint8_t applicationData[
        UartFraming::MAX_APPLICATION_FRAME_SIZE
    ] = {};
    size_t applicationLength = 0;

    const UartFraming::DecodeStatus status =
        UartFraming::decodeApplicationFrame(
            encodedData,
            encodedLength,
            applicationData,
            sizeof(applicationData),
            applicationLength
        );

    if (status != UartFraming::DecodeStatus::OK)
    {
        Serial.printf(
            "STM32 transport frame rejected: %s\n",
            UartFraming::decodeStatusName(status)
        );
        return;
    }

    if (receiveCallback != nullptr)
    {
        receiveCallback(
            applicationData,
            applicationLength
        );
    }
}

}


bool stm32TransportInit()
{
    stm32Serial.begin(
        AppConfig::Stm32Uart::BAUD_RATE,
        SERIAL_8N1,
        AppConfig::Stm32Uart::RX_PIN,
        AppConfig::Stm32Uart::TX_PIN
    );

    streamReceiver.reset();
    transportInitialized = true;

    Serial.println();
    Serial.println("STM32 UART transport initialized");

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


void stm32TransportSetReceiveCallback(
    Stm32TransportReceiveCallback callback
)
{
    receiveCallback = callback;
}


bool stm32TransportSend(
    const uint8_t *data,
    size_t length
)
{
    if (!transportInitialized)
    {
        Serial.println(
            "STM32 UART transport is not initialized"
        );
        return false;
    }

    uint8_t wireFrame[
        UartFraming::MAX_WIRE_FRAME_SIZE
    ] = {};
    size_t wireLength = 0;

    if (
        !UartFraming::encodeApplicationFrame(
            data,
            length,
            wireFrame,
            sizeof(wireFrame),
            wireLength
        )
    )
    {
        Serial.println(
            "STM32 transport frame encoding failed"
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
            "STM32 transport TX incomplete: %u/%u bytes\n",
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
        "STM32 transport TX: wire=%u bytes\n",
        static_cast<unsigned int>(wireLength)
    );

    return true;
}


void stm32TransportPoll()
{
    if (!transportInitialized)
    {
        return;
    }

    while (stm32Serial.available() > 0)
    {
        const uint8_t value =
            static_cast<uint8_t>(
                stm32Serial.read()
            );
        UartStreamFramer::FrameView frame = {};

        const UartStreamFramer::PushStatus status =
            streamReceiver.push(
                value,
                frame
            );

        if (
            status ==
            UartStreamFramer::PushStatus::FRAME_READY
        )
        {
            processEncodedFrame(
                frame.data,
                frame.length
            );
        }
        else if (
            status ==
            UartStreamFramer::PushStatus::FRAME_TOO_LONG
        )
        {
            Serial.println(
                "STM32 transport frame too long, "
                "discarding until delimiter"
            );
        }
    }
}
