#include "websocket_service.h"

#include <Arduino.h>

#include <cstdio>
#include <cstring>

#include "app_config.h"
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


#else


namespace
{

/*
 * 判断一个字符是否应当从命令首尾删除。
 */
bool isTrimCharacter(char value)
{
    return (
        value == ' '  ||
        value == '\t' ||
        value == '\r' ||
        value == '\n'
    );
}


/*
 * 原地删除字符串首尾的空格和换行。
 */
void trimCommand(char *command)
{
    if (command == nullptr)
    {
        return;
    }

    size_t length = strlen(command);
    size_t start = 0;

    while (
        start < length &&
        isTrimCharacter(command[start])
    )
    {
        start++;
    }

    if (start > 0)
    {
        memmove(
            command,
            command + start,
            length - start + 1
        );

        length -= start;
    }

    while (
        length > 0 &&
        isTrimCharacter(
            command[length - 1]
        )
    )
    {
        command[length - 1] = '\0';
        length--;
    }
}


/*
 * 检查 #MOVE 命令是否合法。
 *
 * 合法示例：
 *
 * #MOVE,1,-2
 */
bool validateMoveCommand(
    const char *command
)
{
    int yawStep = 0;
    int pitchStep = 0;

    /*
     * extra 用来检查命令结尾是否还有多余字符。
     *
     * 正常命令只有两个整数，因此 sscanf
     * 应当只成功转换两个字段。
     */
    char extra = '\0';

    const int matched = sscanf(
        command,
        "#MOVE,%d,%d%c",
        &yawStep,
        &pitchStep,
        &extra
    );

    if (matched != 2)
    {
        return false;
    }

    if (
        yawStep <
            AppConfig::WebSocket::MIN_MOVE_STEP ||
        yawStep >
            AppConfig::WebSocket::MAX_MOVE_STEP
    )
    {
        return false;
    }

    if (
        pitchStep <
            AppConfig::WebSocket::MIN_MOVE_STEP ||
        pitchStep >
            AppConfig::WebSocket::MAX_MOVE_STEP
    )
    {
        return false;
    }

    return true;
}


/*
 * 只允许转发已经定义好的 STM32 命令。
 *
 * 不允许网络客户端向 STM32 随意发送任意字符串。
 */
bool validateCommand(
    const char *command
)
{
    if (
        command == nullptr ||
        command[0] == '\0'
    )
    {
        return false;
    }

    if (strcmp(command, "#STOP") == 0)
    {
        return true;
    }

    if (strcmp(command, "#CENTER") == 0)
    {
        return true;
    }

    if (strcmp(command, "#GET") == 0)
    {
        return true;
    }

    if (
        strncmp(
            command,
            "#MOVE,",
            strlen("#MOVE,")
        ) == 0
    )
    {
        return validateMoveCommand(command);
    }

    return false;
}


/*
 * WebSocket 的处理函数。
 */
esp_err_t websocketHandler(
    httpd_req_t *request
)
{
    /*
     * 第一次访问 /ws 时是 HTTP Upgrade 握手。
     *
     * esp_http_server 会处理握手，
     * Handler 这里只需要返回成功。
     */
    if (request->method == HTTP_GET)
    {
        Serial.println(
            "WebSocket client connected"
        );

        return ESP_OK;
    }

    httpd_ws_frame_t frame = {};

    /*
     * 第一次调用不读取数据，只查询：
     *
     * - 帧类型
     * - 数据长度
     */
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
        AppConfig::WebSocket::MAX_COMMAND_LENGTH
    )
    {
        Serial.printf(
            "WebSocket command too long: %u bytes\n",
            static_cast<unsigned int>(
                frame.len
            )
        );

        /*
         * 返回 ESP_FAIL 会关闭这个异常连接。
         */
        return ESP_FAIL;
    }

    uint8_t payload[
        AppConfig::WebSocket::MAX_COMMAND_LENGTH + 1
    ] = {};

    frame.payload = payload;

    /*
     * 第二次调用读取实际负载。
     */
    result =
        httpd_ws_recv_frame(
            request,
            &frame,
            AppConfig::WebSocket::
                MAX_COMMAND_LENGTH
        );

    if (result != ESP_OK)
    {
        Serial.printf(
            "WebSocket payload receive failed: 0x%x\n",
            result
        );

        return result;
    }

    /*
     * 把收到的字节补成标准 C 字符串。
     */
    payload[frame.len] = '\0';


    /*
     * 响应客户端的 WebSocket Ping。
     */
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
        Serial.println(
            "WebSocket client requested close"
        );

        return ESP_OK;
    }

    /*
     * 云台控制只接收文本帧。
     */
    if (frame.type != HTTPD_WS_TYPE_TEXT)
    {
        Serial.printf(
            "Unsupported WebSocket frame type: %d\n",
            static_cast<int>(
                frame.type
            )
        );

        return ESP_OK;
    }

    char *command =
        reinterpret_cast<char *>(
            payload
        );

    trimCommand(command);

    Serial.printf(
        "WebSocket RX: %s\n",
        command
    );

    if (!validateCommand(command))
    {
        Serial.println(
            "Invalid WebSocket command, ignored"
        );

        return ESP_OK;
    }

    /*
     * 命令合法，原样转发给 STM32。
     */
    if (!stm32UartSendCommand(command))
    {
        Serial.println(
            "Failed to forward command to STM32"
        );

        return ESP_FAIL;
    }

    /*
     * 当前不向 Linux 返回 ACK。
     *
     * ROS 2 客户端目前只发送、不持续接收；
     * 因此避免不断产生无人读取的返回消息。
     */
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

    /*
     * 这一项告诉 esp_http_server：
     *
     * /ws 不是普通 GET 接口，
     * 而是 WebSocket Upgrade 接口。
     */
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

    Serial.println(
        "WebSocket service registered: /ws"
    );

    return true;
}


#endif