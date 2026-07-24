#pragma once

#include "esp_http_server.h"


/*
 * 在现有 HTTP 服务器中注册：
 *
 * WebSocket /ws
 *
 * 每个二进制 WebSocket 帧承载一条完整
 * V1 应用消息。
 *
 * ws://ESP32_IP/ws
 */
bool websocketServiceRegister(
    httpd_handle_t server
);
