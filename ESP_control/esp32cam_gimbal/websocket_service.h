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


/*
 * 当前是否存在仍由 HTTP 服务器识别为 WebSocket 的 Linux 客户端。
 *
 * UDP 发现服务使用这个状态：连接建立后暂停广播，断开后恢复。
 */
bool websocketServiceHasClient();
