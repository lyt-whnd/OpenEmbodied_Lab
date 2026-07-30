#include "websocket_transport.h"

#include <Arduino.h>

#include <atomic>

#include "protocol_v1.h"


#ifndef CONFIG_HTTPD_WS_SUPPORT


void websocketTransportSetReceiveCallback(
    WebsocketTransportReceiveCallback callback
)
{
    (void)callback;
}


bool websocketTransportRegister(
    httpd_handle_t server
)
{
    (void)server;

    Serial.println(
        "WebSocket is not enabled in this ESP32 core"
    );

    return false;
}


bool websocketTransportSend(
    const uint8_t *data,
    size_t length
)
{
    (void)data;
    (void)length;
    return false;
}


bool websocketTransportHasClient()
{
    return false;
}


#else


namespace
{

httpd_handle_t websocketServer = nullptr;
std::atomic<int> websocketClientFd(-1);
WebsocketTransportReceiveCallback receiveCallback =
    nullptr;


void clearClientIfCurrent(
    int clientFd
)
{
    int expectedFd = clientFd;

    websocketClientFd.compare_exchange_strong(
        expectedFd,
        -1
    );
}


bool clientIsReady(
    int &clientFd
)
{
    clientFd = websocketClientFd.load();

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
        clearClientIfCurrent(clientFd);
    }

    return ready;
}


esp_err_t websocketHandler(
    httpd_req_t *request
)
{
    if (request->method == HTTP_GET)
    {
        const int newClientFd =
            httpd_req_to_sockfd(request);
        const int oldClientFd =
            websocketClientFd.exchange(
                newClientFd
            );

        Serial.printf(
            "V1 WebSocket client connected: fd=%d\n",
            newClientFd
        );

        if (
            oldClientFd >= 0 &&
            oldClientFd != newClientFd
        )
        {
            Serial.printf(
                "Replacing V1 WebSocket client: "
                "old_fd=%d, new_fd=%d\n",
                oldClientFd,
                newClientFd
            );

            httpd_sess_trigger_close(
                websocketServer,
                oldClientFd
            );
        }

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
            "WebSocket frame too long: %u bytes\n",
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

        clearClientIfCurrent(clientFd);

        Serial.printf(
            "V1 WebSocket client requested close: "
            "fd=%d\n",
            clientFd
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

    if (receiveCallback != nullptr)
    {
        receiveCallback(
            payload,
            frame.len
        );
    }

    return ESP_OK;
}

}


void websocketTransportSetReceiveCallback(
    WebsocketTransportReceiveCallback callback
)
{
    receiveCallback = callback;
}


bool websocketTransportRegister(
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
    websocketUri.handler = websocketHandler;
    websocketUri.user_ctx = nullptr;
    websocketUri.is_websocket = true;

    if (
        httpd_register_uri_handler(
            server,
            &websocketUri
        ) != ESP_OK
    )
    {
        Serial.println(
            "Cannot register WebSocket URI"
        );

        return false;
    }

    websocketServer = server;
    websocketClientFd.store(-1);

    Serial.println(
        "V1 WebSocket transport: /ws"
    );

    return true;
}


bool websocketTransportSend(
    const uint8_t *data,
    size_t length
)
{
    if (
        data == nullptr ||
        length == 0U ||
        length > ProtocolV1::MAX_MESSAGE_SIZE
    )
    {
        return false;
    }

    int clientFd = -1;

    if (!clientIsReady(clientFd))
    {
        return false;
    }

    httpd_ws_frame_t frame = {};

    frame.type = HTTPD_WS_TYPE_BINARY;
    frame.payload =
        const_cast<uint8_t *>(data);
    frame.len = length;

    const esp_err_t result =
        httpd_ws_send_frame_async(
            websocketServer,
            clientFd,
            &frame
        );

    if (result != ESP_OK)
    {
        clearClientIfCurrent(clientFd);

        Serial.printf(
            "WebSocket binary send failed: 0x%x\n",
            result
        );

        return false;
    }

    return true;
}


bool websocketTransportHasClient()
{
    int clientFd = -1;
    return clientIsReady(clientFd);
}


#endif
