#include "wifi_service.h"

#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "secrets.h"


bool wifiServiceConnect()
{
    Serial.println();
    Serial.printf(
        "Connecting to WiFi: %s\n",
        Secrets::WIFI_SSID
    );

    /*
     * STA 模式：
     * ESP32 作为普通终端连接路由器。
     */
    WiFi.mode(WIFI_STA);

    WiFi.begin(
        Secrets::WIFI_SSID,
        Secrets::WIFI_PASSWORD
    );

    const unsigned long startTime =
        millis();

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");

        if (
            millis() - startTime >=
            AppConfig::WIFI_TIMEOUT_MS
        )
        {
            Serial.println();
            Serial.println(
                "WiFi connection timeout"
            );

            return false;
        }
    }

    Serial.println();
    Serial.println("WiFi connected");

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    return true;
}