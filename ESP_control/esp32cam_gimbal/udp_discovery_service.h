#pragma once


/*
 * 打开 UDP 广播端点并立即发送第一条 ROBOT_HELLO。
 */
bool udpDiscoveryStart();


/*
 * 非阻塞轮询；连接 Wi-Fi 时每秒广播一次发现消息。
 */
void udpDiscoveryPoll();


/*
 * 关闭 UDP 端点。
 */
void udpDiscoveryStop();
