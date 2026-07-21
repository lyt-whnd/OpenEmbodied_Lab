#pragma once


/*
 * 启动网络服务。
 *
 * 80端口：
 *
 *   GET /
 *   GET /capture
 *   WebSocket /ws
 *
 * 81端口：
 *
 *   GET /stream
 */
bool httpServiceStart();