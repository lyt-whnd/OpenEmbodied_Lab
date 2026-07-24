#pragma once

#include <stddef.h>
#include <stdint.h>


using Stm32UartMessageCallback = void (*)(
    const uint8_t *message,
    size_t length
);


/*
 * 初始化 ESP32-CAM 与 STM32 之间的硬件串口。
 */
bool stm32UartInit();


/*
 * 注册一个有效 STM32 V1 消息的接收回调。
 *
 * message 指针只在回调执行期间有效。
 */
void stm32UartSetMessageCallback(
    Stm32UartMessageCallback callback
);


/*
 * 使用 COBS + CRC16-CCITT-FALSE 封装并发送
 * 一条完整的 V1 应用消息。
 */
bool stm32UartSendApplicationMessage(
    const uint8_t *message,
    size_t length
);


/*
 * 非阻塞读取 UART 字节流，按 0x00 分帧，
 * 完成 COBS、CRC16 和 V1 应用头校验。
 */
void stm32UartPoll();
