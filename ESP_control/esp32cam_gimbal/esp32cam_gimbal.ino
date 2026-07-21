#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "camera_service.h"
#include "wifi_service.h"
#include "http_service.h"

#include "stm32_uart.h"


/*
 * 严重错误发生后停止程序。
 */
void stopProgram(const char *reason)
{
    Serial.println();
    Serial.print("Program stopped: ");
    Serial.println(reason);

    while (true)
    {
        delay(1000);
    }
}


void setup()
{
    Serial.begin(
        AppConfig::DEBUG_BAUD
    );

    delay(1500);

    Serial.println();
    Serial.println("==============================");
    Serial.println("ESP32-CAM gimbal system");
    Serial.println("==============================");

    /*
     * 1. 初始化摄像头。
     */
    if (!cameraServiceInit())
    {
        stopProgram(
            "camera initialization failed"
        );
    }

    /*
    * 2. 初始化 ESP32-CAM 与 STM32 的 UART。
    */
    if (!stm32UartInit())
    {
        stopProgram(
            "STM32 UART initialization failed"
        );
    }

    /*
     * 2. 连接 Wi-Fi。
     */
    if (!wifiServiceConnect())
    {
        stopProgram(
            "WiFi connection failed"
        );
    }

    /*
     * 3. 启动网页和视频流服务。
     */
    if (!httpServiceStart())
    {
        stopProgram(
            "HTTP service failed"
        );
    }

    Serial.println();
    Serial.println("==============================");

    Serial.print("Web page: http://");
    Serial.println(WiFi.localIP());

    Serial.print("JPEG:     http://");
    Serial.print(WiFi.localIP());
    Serial.println("/capture");

    Serial.print("Stream:   http://");
    Serial.print(WiFi.localIP());
    Serial.println(":81/stream");

    Serial.print("WebSocket: ws://");
    Serial.print(WiFi.localIP());
    Serial.println("/ws");

    Serial.println("==============================");
}


void loop()
{
    /*
     * 检查 STM32 是否返回 ACK 或状态信息。
     */
    stm32UartPoll();
    /*
     * 不使用长时间 delay，
     * 避免 STM32 返回数据积压。
     */

    /*
     * HTTP服务器运行在自己的任务中。
     */
    delay(10);
}