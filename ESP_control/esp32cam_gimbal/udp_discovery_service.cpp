#include "udp_discovery_service.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "app_config.h"


namespace
{

WiFiUDP discoveryUdp;
bool discoveryStarted = false;
bool broadcastSuppressed = false;
unsigned long lastBroadcastAt = 0;


IPAddress directedBroadcastAddress()
{
    const IPAddress localIp = WiFi.localIP();
    const IPAddress subnetMask = WiFi.subnetMask();

    return IPAddress(
        localIp[0] | static_cast<uint8_t>(~subnetMask[0]),
        localIp[1] | static_cast<uint8_t>(~subnetMask[1]),
        localIp[2] | static_cast<uint8_t>(~subnetMask[2]),
        localIp[3] | static_cast<uint8_t>(~subnetMask[3])
    );
}


bool sendDiscoveryPacket()
{
    if (
        !discoveryStarted ||
        WiFi.status() != WL_CONNECTED
    )
    {
        return false;
    }

    const uint32_t chipSuffix =
        static_cast<uint32_t>(ESP.getEfuseMac()) &
        0xFFFFFF;
    const String localIp = WiFi.localIP().toString();
    char deviceId[24];
    char payload[384];

    snprintf(
        deviceId,
        sizeof(deviceId),
        "ESP32CAM_%06X",
        chipSuffix
    );

    const int payloadLength = snprintf(
        payload,
        sizeof(payload),
        "{"
        "\"magic\":\"%s\","
        "\"proto\":1,"
        "\"name\":\"%s\","
        "\"node\":\"%s\","
        "\"device_id\":\"%s\","
        "\"ip\":\"%s\","
        "\"ws_port\":%u,"
        "\"ws_path\":\"%s\","
        "\"tcp_port\":%u,"
        "\"stream_port\":%u,"
        "\"stream_path\":\"%s\""
        "}",
        AppConfig::Discovery::MAGIC,
        AppConfig::Discovery::ROBOT_NAME,
        AppConfig::Discovery::NODE_NAME,
        deviceId,
        localIp.c_str(),
        static_cast<unsigned int>(AppConfig::HTTP_PORT),
        AppConfig::Discovery::WEBSOCKET_PATH,
        static_cast<unsigned int>(
            AppConfig::CONTROL_TCP_PORT
        ),
        static_cast<unsigned int>(AppConfig::STREAM_PORT),
        AppConfig::Discovery::STREAM_PATH
    );

    if (
        payloadLength <= 0 ||
        static_cast<size_t>(payloadLength) >= sizeof(payload)
    )
    {
        Serial.println("UDP discovery payload overflow");
        return false;
    }

    const IPAddress broadcastIp = directedBroadcastAddress();

    if (
        !discoveryUdp.beginPacket(
            broadcastIp,
            AppConfig::Discovery::PORT
        )
    )
    {
        Serial.println("Cannot begin UDP discovery packet");
        return false;
    }

    const size_t bytesWritten = discoveryUdp.write(
        reinterpret_cast<const uint8_t *>(payload),
        static_cast<size_t>(payloadLength)
    );
    const bool sent =
        bytesWritten == static_cast<size_t>(payloadLength) &&
        discoveryUdp.endPacket() == 1;

    if (!sent)
    {
        Serial.println("UDP discovery broadcast failed");
        return false;
    }

    lastBroadcastAt = millis();
    return true;
}

}  // namespace


bool udpDiscoveryStart()
{
    if (discoveryStarted)
    {
        return true;
    }

    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println(
            "Cannot start UDP discovery before WiFi connects"
        );
        return false;
    }

    /*
     * 使用端口 0 让网络栈选择临时源端口。目标端口始终为 4210。
     */
    if (!discoveryUdp.begin(0))
    {
        Serial.println("Cannot open UDP discovery socket");
        return false;
    }

    discoveryStarted = true;
    broadcastSuppressed = false;
    lastBroadcastAt = 0;

    Serial.printf(
        "UDP discovery broadcasting on port %u\n",
        static_cast<unsigned int>(
            AppConfig::Discovery::PORT
        )
    );

    /*
     * 第一次发送失败不应阻止机器人其余服务启动；poll 会继续重试。
     */
    sendDiscoveryPacket();
    return true;
}


void udpDiscoveryPoll(
    bool linuxControlConnected
)
{
    if (
        !discoveryStarted ||
        WiFi.status() != WL_CONNECTED
    )
    {
        return;
    }

    if (linuxControlConnected)
    {
        if (!broadcastSuppressed)
        {
            broadcastSuppressed = true;
            Serial.println(
                "UDP discovery paused: Linux control connected"
            );
        }

        return;
    }

    if (broadcastSuppressed)
    {
        broadcastSuppressed = false;

        /*
         * 断线后不再等待一个完整周期，下一段代码会立即广播。
         */
        lastBroadcastAt =
            millis() -
            AppConfig::Discovery::INTERVAL_MS;

        Serial.println(
            "UDP discovery resumed: Linux control disconnected"
        );
    }

    if (
        millis() - lastBroadcastAt >=
        AppConfig::Discovery::INTERVAL_MS
    )
    {
        sendDiscoveryPacket();
    }
}


void udpDiscoveryStop()
{
    if (!discoveryStarted)
    {
        return;
    }

    discoveryUdp.stop();
    discoveryStarted = false;
    broadcastSuppressed = false;
}
