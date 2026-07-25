#include "websocket_service.h"

#include <Arduino.h>

#include <atomic>
#include <string.h>

#include "protocol_v1.h"
#include "stm32_uart.h"


#ifndef CONFIG_HTTPD_WS_SUPPORT


bool websocketServiceRegister(
    httpd_handle_t server
)
{
    (void)server;

    Serial.println(
        "WebSocket is not enabled in this ESP32 core"
    );

    return false;
}


bool websocketServiceHasClient()
{
    return false;
}


#else


namespace
{

httpd_handle_t websocketServer = nullptr;
std::atomic<int> websocketClientFd(-1);

uint8_t websocketTxBuffer[
    ProtocolV1::MAX_MESSAGE_SIZE
];


bool websocketClientIsReady()
{
    const int clientFd = websocketClientFd.load();

    if (
        websocketServer == nullptr ||
        clientFd < 0
    )
    {
        return false;
    }

    const bool ready = (
        httpd_ws_get_fd_info(
            websocketServer,
            clientFd
        ) ==
        HTTPD_WS_CLIENT_WEBSOCKET
    );

    if (!ready)
    {
        int expectedFd = clientFd;

        websocketClientFd.compare_exchange_strong(
            expectedFd,
            -1
        );
    }

    return ready;
}


bool sendBinaryFrame(
    httpd_req_t *request,
    const uint8_t *data,
    size_t length
)
{
    httpd_ws_frame_t response = {};

    response.type = HTTPD_WS_TYPE_BINARY;
    response.payload =
        const_cast<uint8_t *>(data);
    response.len = length;

    return (
        httpd_ws_send_frame(
            request,
            &response
        ) == ESP_OK
    );
}


bool handleLocalMessage(
    httpd_req_t *request,
    const ProtocolV1::MessageView &message
)
{
    if (
        message.service !=
            ProtocolV1::SERVICE_SYSTEM ||
        message.opcode !=
            ProtocolV1::SYSTEM_PING
    )
    {
        Serial.printf(
            "ESP32 local V1 message unsupported: "
            "service=0x%02X, opcode=0x%02X\n",
            static_cast<unsigned int>(
                message.service
            ),
            static_cast<unsigned int>(
                message.opcode
            )
        );

        return false;
    }

    ProtocolV1::MessageView pong = {};

    pong.version = ProtocolV1::VERSION;
    pong.flags = ProtocolV1::FLAG_RESPONSE;
    pong.src = ProtocolV1::NODE_ESP32;
    pong.dst = ProtocolV1::NODE_LINUX;
    pong.service = ProtocolV1::SERVICE_SYSTEM;
    pong.opcode = ProtocolV1::SYSTEM_PONG;
    pong.seq = message.seq;
    pong.payloadLength = 0;
    pong.payload = nullptr;

    size_t responseLength = 0;

    if (
        !ProtocolV1::encodeMessage(
            pong,
            websocketTxBuffer,
            sizeof(websocketTxBuffer),
            responseLength
        )
    )
    {
        Serial.println(
            "Cannot encode SYSTEM/PONG"
        );

        return false;
    }

    if (
        !sendBinaryFrame(
            request,
            websocketTxBuffer,
            responseLength
        )
    )
    {
        Serial.println(
            "Cannot send SYSTEM/PONG"
        );

        return false;
    }

    Serial.printf(
        "WebSocket TX V1: SYSTEM/PONG seq=%u\n",
        static_cast<unsigned int>(
            message.seq
        )
    );

    return true;
}


void forwardStm32MessageToLinux(
    const uint8_t *messageData,
    size_t length
)
{
    ProtocolV1::MessageView message = {};

    const ProtocolV1::DecodeStatus status =
        ProtocolV1::decodeMessage(
            messageData,
            length,
            message
        );

    if (status != ProtocolV1::DecodeStatus::OK)
    {
        Serial.printf(
            "STM32 to WebSocket rejected: %s\n",
            ProtocolV1::decodeStatusName(status)
        );

        return;
    }

    if (
        message.dst != ProtocolV1::NODE_LINUX &&
        message.dst != ProtocolV1::NODE_BROADCAST
    )
    {
        /*
         * 发给 ESP32 本机的 STM32 消息留给后续
         * 本地服务处理器，不转发给 Linux。
         */
        return;
    }

    if (!websocketClientIsReady())
    {
        Serial.println(
            "STM32 V1 message dropped: "
            "Linux WebSocket is offline"
        );

        return;
    }

    memcpy(
        websocketTxBuffer,
        messageData,
        length
    );

    httpd_ws_frame_t frame = {};

    frame.type = HTTPD_WS_TYPE_BINARY;
    frame.payload = websocketTxBuffer;
    frame.len = length;

    const esp_err_t result =
        httpd_ws_send_frame_async(
            websocketServer,
            websocketClientFd.load(),
            &frame
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "STM32 to WebSocket send failed: 0x%x\n",
            result
        );

        return;
    }

    Serial.printf(
        "WebSocket TX V1: seq=%u, "
        "service=0x%02X, opcode=0x%02X\n",
        static_cast<unsigned int>(message.seq),
        static_cast<unsigned int>(
            message.service
        ),
        static_cast<unsigned int>(
            message.opcode
        )
    );
}


esp_err_t websocketHandler(
    httpd_req_t *request
)
{
    if (request->method == HTTP_GET)
    {
        websocketClientFd.store(
            httpd_req_to_sockfd(request)
        );

        Serial.printf(
            "V1 WebSocket client connected: fd=%d\n",
            websocketClientFd.load()
        );

        return ESP_OK;
    }

    httpd_ws_frame_t frame = {};

    esp_err_t result =
        httpd_ws_recv_frame(
            request,
            &frame,
            0
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "WebSocket frame header failed: 0x%x\n",
            result
        );

        return result;
    }

    if (
        frame.len >
        ProtocolV1::MAX_MESSAGE_SIZE
    )
    {
        Serial.printf(
            "WebSocket V1 frame too long: %u bytes\n",
            static_cast<unsigned int>(
                frame.len
            )
        );

        return ESP_FAIL;
    }

    uint8_t payload[
        ProtocolV1::MAX_MESSAGE_SIZE
    ] = {};

    frame.payload = payload;

    result =
        httpd_ws_recv_frame(
            request,
            &frame,
            sizeof(payload)
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "WebSocket payload receive failed: 0x%x\n",
            result
        );

        return result;
    }

    if (frame.type == HTTPD_WS_TYPE_PING)
    {
        frame.type = HTTPD_WS_TYPE_PONG;

        return httpd_ws_send_frame(
            request,
            &frame
        );
    }

    if (frame.type == HTTPD_WS_TYPE_PONG)
    {
        return ESP_OK;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE)
    {
        const int clientFd =
            httpd_req_to_sockfd(request);

        int expectedFd = clientFd;

        websocketClientFd.compare_exchange_strong(
            expectedFd,
            -1
        );

        Serial.println(
            "V1 WebSocket client requested close"
        );

        return ESP_OK;
    }

    if (frame.type != HTTPD_WS_TYPE_BINARY)
    {
        Serial.printf(
            "Unsupported WebSocket frame type: %d\n",
            static_cast<int>(frame.type)
        );

        return ESP_OK;
    }

    ProtocolV1::MessageView message = {};

    const ProtocolV1::DecodeStatus status =
        ProtocolV1::decodeMessage(
            payload,
            frame.len,
            message
        );

    if (status != ProtocolV1::DecodeStatus::OK)
    {
        Serial.printf(
            "WebSocket V1 message rejected: %s\n",
            ProtocolV1::decodeStatusName(status)
        );

        return ESP_OK;
    }

    if (message.src != ProtocolV1::NODE_LINUX)
    {
        Serial.printf(
            "WebSocket V1 rejected: invalid src=0x%02X\n",
            static_cast<unsigned int>(
                message.src
            )
        );

        return ESP_OK;
    }

    if (
        message.dst != ProtocolV1::NODE_ESP32 &&
        message.dst != ProtocolV1::NODE_STM32 &&
        message.dst != ProtocolV1::NODE_BROADCAST
    )
    {
        Serial.printf(
            "WebSocket V1 rejected: invalid dst=0x%02X\n",
            static_cast<unsigned int>(
                message.dst
            )
        );

        return ESP_OK;
    }

    Serial.printf(
        "WebSocket RX V1: seq=%u, "
        "src=0x%02X, dst=0x%02X, "
        "service=0x%02X, opcode=0x%02X, "
        "payload=%u\n",
        static_cast<unsigned int>(message.seq),
        static_cast<unsigned int>(message.src),
        static_cast<unsigned int>(message.dst),
        static_cast<unsigned int>(
            message.service
        ),
        static_cast<unsigned int>(
            message.opcode
        ),
        static_cast<unsigned int>(
            message.payloadLength
        )
    );

    bool handled = false;

    if (
        message.dst == ProtocolV1::NODE_ESP32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled =
            handleLocalMessage(
                request,
                message
            ) ||
            handled;
    }

    if (
        message.dst == ProtocolV1::NODE_STM32 ||
        message.dst == ProtocolV1::NODE_BROADCAST
    )
    {
        handled =
            stm32UartSendApplicationMessage(
                payload,
                frame.len
            ) ||
            handled;
    }

    if (!handled)
    {
        Serial.println(
            "WebSocket V1 message was not handled"
        );
    }

    return ESP_OK;
}

}


bool websocketServiceRegister(
    httpd_handle_t server
)
{
    if (server == nullptr)
    {
        Serial.println(
            "Cannot register WebSocket: "
            "HTTP server is null"
        );

        return false;
    }

    httpd_uri_t websocketUri = {};

    websocketUri.uri = "/ws";
    websocketUri.method = HTTP_GET;
    websocketUri.handler =
        websocketHandler;
    websocketUri.user_ctx = nullptr;
    websocketUri.is_websocket = true;

    const esp_err_t result =
        httpd_register_uri_handler(
            server,
            &websocketUri
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "Register /ws failed: 0x%x\n",
            result
        );

        return false;
    }

    websocketServer = server;
    websocketClientFd.store(-1);

    stm32UartSetMessageCallback(
        forwardStm32MessageToLinux
    );

    Serial.println(
        "V1 binary WebSocket service registered: /ws"
    );

    return true;
}


bool websocketServiceHasClient()
{
    return websocketClientIsReady();
}


#endif
