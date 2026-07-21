#pragma once

#include "esp_http_server.h"


/*
 * 在现有 HTTP 服务器中注册：
 *
 * GET /ws
 *
 * 客户端地址：
 *
 * ws://ESP32_IP/ws
 */
bool websocketServiceRegister(
    httpd_handle_t server
);