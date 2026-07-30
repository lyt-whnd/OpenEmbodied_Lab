#include "tcp_transport.h"

#include <Arduino.h>
#include <WiFi.h>

#include "app_config.h"
#include "tcp_packet_framer.h"
#include "transport_limits.h"


namespace
{

WiFiServer tcpServer(
    AppConfig::CONTROL_TCP_PORT
);
WiFiClient tcpClient;
TcpPacketFraming::StreamDecoder decoder;
TcpTransportReceiveCallback receiveCallback =
    nullptr;
bool serverStarted = false;


void closeClient()
{
    if (tcpClient)
    {
        tcpClient.stop();
    }

    tcpClient = WiFiClient();
    decoder.reset();
}


void acceptClientIfNeeded()
{
    if (
        tcpClient &&
        tcpClient.connected()
    )
    {
        return;
    }

    closeClient();

    WiFiClient candidate =
        tcpServer.available();

    if (!candidate)
    {
        return;
    }

    tcpClient = candidate;
    tcpClient.setNoDelay(true);
    decoder.reset();

    Serial.printf(
        "V1 TCP client connected: %s:%u\n",
        tcpClient.remoteIP().toString().c_str(),
        static_cast<unsigned int>(
            tcpClient.remotePort()
        )
    );
}

}


bool tcpTransportBegin()
{
    if (serverStarted)
    {
        return true;
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);
    serverStarted = true;

    Serial.printf(
        "V1 TCP fallback transport: port %u\n",
        static_cast<unsigned int>(
            AppConfig::CONTROL_TCP_PORT
        )
    );

    return true;
}


void tcpTransportSetReceiveCallback(
    TcpTransportReceiveCallback callback
)
{
    receiveCallback = callback;
}


bool tcpTransportSend(
    const uint8_t *data,
    size_t length
)
{
    if (
        data == nullptr ||
        length == 0U ||
        length > TransportLimits::MAX_OPAQUE_FRAME_SIZE ||
        !tcpTransportHasClient()
    )
    {
        return false;
    }

    uint8_t prefix[
        TcpPacketFraming::LENGTH_PREFIX_SIZE
    ] = {};

    if (
        !TcpPacketFraming::encodeLengthPrefix(
            length,
            prefix
        )
    )
    {
        return false;
    }

    const size_t prefixWritten =
        tcpClient.write(
            prefix,
            sizeof(prefix)
        );
    const size_t payloadWritten =
        tcpClient.write(
            data,
            length
        );

    if (
        prefixWritten != sizeof(prefix) ||
        payloadWritten != length
    )
    {
        Serial.println(
            "V1 TCP send incomplete; closing client"
        );
        closeClient();
        return false;
    }

    return true;
}


bool tcpTransportHasClient()
{
    if (
        !serverStarted ||
        !tcpClient ||
        !tcpClient.connected()
    )
    {
        if (tcpClient)
        {
            closeClient();
        }
        return false;
    }

    return true;
}


void tcpTransportPoll()
{
    if (!serverStarted)
    {
        return;
    }

    acceptClientIfNeeded();

    while (
        tcpTransportHasClient() &&
        tcpClient.available() > 0
    )
    {
        const int input = tcpClient.read();

        if (input < 0)
        {
            break;
        }

        TcpPacketFraming::FrameView frame = {};
        const TcpPacketFraming::PushStatus status =
            decoder.push(
                static_cast<uint8_t>(input),
                frame
            );

        if (
            status ==
            TcpPacketFraming::PushStatus::INVALID_LENGTH
        )
        {
            Serial.println(
                "Invalid V1 TCP packet length; "
                "closing client"
            );
            closeClient();
            return;
        }

        if (
            status ==
                TcpPacketFraming::PushStatus::FRAME_READY &&
            receiveCallback != nullptr
        )
        {
            receiveCallback(
                frame.data,
                frame.length
            );
        }
    }
}
