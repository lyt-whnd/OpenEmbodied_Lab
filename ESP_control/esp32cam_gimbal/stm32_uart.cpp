#include "stm32_uart.h"

#include <Arduino.h>

#include "app_config.h"


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


/*
 * 接收 STM32 返回字符串的缓冲区。
 */
char receiveLine[
    AppConfig::Stm32Uart::RX_LINE_MAX_LENGTH
];

size_t receiveLength = 0;

}


bool stm32UartInit()
{
    stm32Serial.begin(
        AppConfig::Stm32Uart::BAUD_RATE,
        SERIAL_8N1,
        AppConfig::Stm32Uart::RX_PIN,
        AppConfig::Stm32Uart::TX_PIN
    );

    receiveLength = 0;
    uartInitialized = true;

    Serial.println();
    Serial.println("STM32 UART initialized");

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


bool stm32UartSendCommand(
    const char *command
)
{
    if (!uartInitialized)
    {
        Serial.println(
            "STM32 UART is not initialized"
        );

        return false;
    }

    if (
        command == nullptr ||
        command[0] == '\0'
    )
    {
        Serial.println(
            "Cannot send empty STM32 command"
        );

        return false;
    }

    /*
     * 先发送命令内容。
     */
    stm32Serial.print(command);

    /*
     * UART 是字节流，没有 WebSocket 的消息边界。
     *
     * 所以使用换行符告诉 STM32：
     * 一条命令到这里结束。
     */
    stm32Serial.write('\n');

    Serial.printf(
        "STM32 TX: %s\n",
        command
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
        const char value =
            static_cast<char>(
                stm32Serial.read()
            );

        /*
         * 忽略 \r，只使用 \n 作为行结束符。
         */
        if (value == '\r')
        {
            continue;
        }

        if (value == '\n')
        {
            if (receiveLength > 0)
            {
                receiveLine[receiveLength] =
                    '\0';

                Serial.printf(
                    "STM32 RX: %s\n",
                    receiveLine
                );

                receiveLength = 0;
            }

            continue;
        }

        /*
         * 留出一个位置存放字符串结束符 '\0'。
         */
        if (
            receiveLength + 1 <
            sizeof(receiveLine)
        )
        {
            receiveLine[receiveLength] =
                value;

            receiveLength++;
        }
        else
        {
            Serial.println(
                "STM32 RX line is too long, discarded"
            );

            receiveLength = 0;
        }
    }
}