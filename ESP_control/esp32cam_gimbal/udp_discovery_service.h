#pragma once


/*
 * 打开 UDP 广播端点并立即发送第一条 ROBOT_HELLO。
 */
bool udpDiscoveryStart();


/*
 * 非阻塞轮询。
 *
 * Linux 控制链路未连接时每秒广播；连接后暂停广播，断开后立即
 * 恢复。只暂停发送，不关闭 UDP socket。
 */
void udpDiscoveryPoll(
    bool linuxControlConnected
);


/*
 * 关闭 UDP 端点。
 */
void udpDiscoveryStop();
