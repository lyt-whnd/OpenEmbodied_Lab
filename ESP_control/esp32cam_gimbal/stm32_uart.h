#pragma once


/*
 * 初始化 ESP32-CAM 与 STM32 之间的硬件串口。
 */
bool stm32UartInit();


/*
 * 向 STM32 发送一条完整命令。
 *
 * 函数会自动在命令末尾补上 '\n'。
 *
 * 例如传入：
 *
 * #MOVE,1,-1
 *
 * STM32 实际收到：
 *
 * #MOVE,1,-1\n
 */
bool stm32UartSendCommand(
    const char *command
);


/*
 * 读取并打印 STM32 返回的信息。
 *
 * 当前主要用于观察 STM32 ACK 和调试输出。
 */
void stm32UartPoll();