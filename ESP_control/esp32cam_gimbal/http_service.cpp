#include "http_service.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>

#include "esp_camera.h"
#include "esp_http_server.h"

#include "app_config.h"

#include "websocket_service.h"


/*
 * 这两个服务器句柄只属于 HTTP 模块。
 */
static httpd_handle_t cameraServer = nullptr;
static httpd_handle_t streamServer = nullptr;


#define STREAM_BOUNDARY \
    "123456789000000000000987654321"


static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary="
    STREAM_BOUNDARY;

static const char *STREAM_FRAME_BOUNDARY =
    "\r\n--"
    STREAM_BOUNDARY
    "\r\n";

static const char *STREAM_FRAME_HEADER =
    "Content-Type: image/jpeg\r\n"
    "Content-Length: %u\r\n"
    "\r\n";


/*
 * GET /
 */
static esp_err_t rootHandler(
    httpd_req_t *request
)
{
    const char *html = R"HTML(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <title>ESP32-CAM Stream</title>
</head>

<body>
    <h1>ESP32-CAM Video Stream</h1>

    <img
        id="cameraStream"
        style="max-width: 100%;"
        alt="ESP32-CAM stream"
    >

    <p>
        <a href="/capture">
            Capture one JPEG
        </a>
    </p>

    <script>
        const streamUrl =
            "http://" +
            window.location.hostname +
            ":81/stream";

        document
            .getElementById("cameraStream")
            .src = streamUrl;
    </script>
</body>
</html>
)HTML";

    httpd_resp_set_type(
        request,
        "text/html; charset=utf-8"
    );

    return httpd_resp_send(
        request,
        html,
        HTTPD_RESP_USE_STRLEN
    );
}


/*
 * GET /capture
 */
static esp_err_t captureHandler(
    httpd_req_t *request
)
{
    Serial.println(
        "HTTP capture request"
    );

    camera_fb_t *frame =
        esp_camera_fb_get();

    if (frame == nullptr)
    {
        Serial.println(
            "Camera capture failed"
        );

        httpd_resp_send_500(request);

        return ESP_FAIL;
    }

    httpd_resp_set_type(
        request,
        "image/jpeg"
    );

    httpd_resp_set_hdr(
        request,
        "Cache-Control",
        "no-store"
    );

    const esp_err_t result =
        httpd_resp_send(
            request,
            reinterpret_cast<const char *>(
                frame->buf
            ),
            frame->len
        );

    esp_camera_fb_return(frame);

    return result;
}


/*
 * GET /stream
 */
static esp_err_t streamHandler(
    httpd_req_t *request
)
{
    Serial.println(
        "Stream client connected"
    );

    esp_err_t result =
        httpd_resp_set_type(
            request,
            STREAM_CONTENT_TYPE
        );

    if (result != ESP_OK)
    {
        return result;
    }

    httpd_resp_set_hdr(
        request,
        "Access-Control-Allow-Origin",
        "*"
    );

    while (true)
    {
        camera_fb_t *frame =
            esp_camera_fb_get();

        if (frame == nullptr)
        {
            Serial.println(
                "Stream capture failed"
            );

            result = ESP_FAIL;
            break;
        }

        if (frame->format != PIXFORMAT_JPEG)
        {
            Serial.println(
                "Frame is not JPEG"
            );

            esp_camera_fb_return(frame);

            result = ESP_FAIL;
            break;
        }

        /*
         * 发送帧边界。
         */
        result =
            httpd_resp_send_chunk(
                request,
                STREAM_FRAME_BOUNDARY,
                strlen(
                    STREAM_FRAME_BOUNDARY
                )
            );

        char frameHeader[80];

        const int headerLength =
            snprintf(
                frameHeader,
                sizeof(frameHeader),
                STREAM_FRAME_HEADER,
                static_cast<unsigned int>(
                    frame->len
                )
            );

        if (
            headerLength < 0 ||
            headerLength >=
                static_cast<int>(
                    sizeof(frameHeader)
                )
        )
        {
            esp_camera_fb_return(frame);

            result = ESP_FAIL;
            break;
        }

        /*
         * 发送本帧 HTTP 头。
         */
        if (result == ESP_OK)
        {
            result =
                httpd_resp_send_chunk(
                    request,
                    frameHeader,
                    static_cast<size_t>(
                        headerLength
                    )
                );
        }

        /*
         * 发送 JPEG 数据。
         */
        if (result == ESP_OK)
        {
            result =
                httpd_resp_send_chunk(
                    request,
                    reinterpret_cast<
                        const char *
                    >(frame->buf),
                    frame->len
                );
        }

        /*
         * 无论成功失败，都归还帧缓冲区。
         */
        esp_camera_fb_return(frame);

        if (result != ESP_OK)
        {
            break;
        }

        delay(1);
    }

    Serial.println(
        "Stream client disconnected"
    );

    return result;
}


/*
 * 启动80端口服务器。
 */
static bool startCameraServer()
{
    httpd_config_t config =
        HTTPD_DEFAULT_CONFIG();

    config.server_port =
        AppConfig::HTTP_PORT;

    const httpd_uri_t rootUri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = rootHandler,
        .user_ctx = nullptr
    };

    const httpd_uri_t captureUri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = captureHandler,
        .user_ctx = nullptr
    };

    esp_err_t error =
        httpd_start(
            &cameraServer,
            &config
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "HTTP server start failed: 0x%x\n",
            error
        );

        return false;
    }

    error =
        httpd_register_uri_handler(
            cameraServer,
            &rootUri
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "Register / failed: 0x%x\n",
            error
        );

        httpd_stop(cameraServer);
        cameraServer = nullptr;

        return false;
    }

    error =
        httpd_register_uri_handler(
            cameraServer,
            &captureUri
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "Register /capture failed: 0x%x\n",
            error
        );

        httpd_stop(cameraServer);
        cameraServer = nullptr;

        return false;
    }

    /*
    * 新增：注册 /ws。
    */
    if (!websocketServiceRegister(cameraServer))
    {
        Serial.println(
            "Register WebSocket service failed"
        );

        httpd_stop(cameraServer);
        cameraServer = nullptr;

        return false;
    }

    Serial.println(
        "HTTP server started on port 80"
    );

    return true;
}


/*
 * 启动81端口视频流服务器。
 */
static bool startStreamServer()
{
    httpd_config_t config =
        HTTPD_DEFAULT_CONFIG();

    config.server_port =
        AppConfig::STREAM_PORT;

    /*
     * 第二个服务器不能与第一个服务器
     * 使用同一个内部控制端口。
     */
    config.ctrl_port += 1;

    const httpd_uri_t streamUri = {
        .uri = "/stream",
        .method = HTTP_GET,
        .handler = streamHandler,
        .user_ctx = nullptr
    };

    esp_err_t error =
        httpd_start(
            &streamServer,
            &config
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "Stream server start failed: 0x%x\n",
            error
        );

        return false;
    }

    error =
        httpd_register_uri_handler(
            streamServer,
            &streamUri
        );

    if (error != ESP_OK)
    {
        Serial.printf(
            "Register /stream failed: 0x%x\n",
            error
        );

        httpd_stop(streamServer);
        streamServer = nullptr;

        return false;
    }

    Serial.println(
        "Stream server started on port 81"
    );

    return true;
}


bool httpServiceStart()
{
    if (!startCameraServer())
    {
        return false;
    }

    if (!startStreamServer())
    {
        httpd_stop(cameraServer);
        cameraServer = nullptr;

        return false;
    }

    return true;
}