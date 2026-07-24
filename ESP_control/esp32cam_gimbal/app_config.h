#pragma once

#include <Arduino.h>


namespace AppConfig
{

    /*
    * 调试串口波特率。
    */
    static constexpr uint32_t DEBUG_BAUD = 115200;

    /*
     * Wi-Fi 自动连接与 SoftAP 配网参数。
     */
    namespace WiFiProvisioning
    {
        static constexpr size_t MAX_SAVED_NETWORKS = 5;

        /*
         * 启动后最多用 30 秒扫描并尝试 NVS 中保存的网络。
         */
        static constexpr uint32_t TOTAL_CONNECT_TIMEOUT_MS = 30000;
        static constexpr uint32_t PER_NETWORK_TIMEOUT_MS = 6000;
        static constexpr uint32_t SCAN_RETRY_DELAY_MS = 1000;

        /*
         * 临时配网热点名称会追加芯片 ID 后六位，
         * 例如 Robot_Config_A1B2C3。
         */
        static constexpr const char *AP_SSID_PREFIX =
            "Robot_Config_";
        static constexpr const char *AP_PASSWORD =
            "robot-config";

        static constexpr uint16_t DNS_PORT = 53;
        static constexpr uint16_t WEB_PORT = 80;
    }

    /*
    * HTTP 服务端口。
    */
    static constexpr uint16_t HTTP_PORT = 80;
    static constexpr uint16_t STREAM_PORT = 81;


    /*
 * ESP32-CAM 与 STM32 的串口配置。
 *
 * GPIO13：ESP32 RX，连接 STM32 TX
 * GPIO14：ESP32 TX，连接 STM32 RX
 *
 * 使用这两个引脚后，不再使用板载 MicroSD。
 */
    namespace Stm32Uart
    {
        static constexpr uint32_t BAUD_RATE = 115200;

        static constexpr int RX_PIN = 13;
        static constexpr int TX_PIN = 14;
    }


    /*
    * AI Thinker ESP32-CAM 摄像头引脚。
    */
    namespace CameraPins
    {
        static constexpr int PWDN  = 32;
        static constexpr int RESET = -1;

        static constexpr int XCLK = 0;
        static constexpr int SIOD = 26;
        static constexpr int SIOC = 27;

        static constexpr int D7 = 35;
        static constexpr int D6 = 34;
        static constexpr int D5 = 39;
        static constexpr int D4 = 36;
        static constexpr int D3 = 21;
        static constexpr int D2 = 19;
        static constexpr int D1 = 18;
        static constexpr int D0 = 5;

        static constexpr int VSYNC = 25;
        static constexpr int HREF  = 23;
        static constexpr int PCLK  = 22;
    }

}
