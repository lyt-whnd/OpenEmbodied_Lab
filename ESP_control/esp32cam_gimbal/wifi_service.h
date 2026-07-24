#pragma once


/*
 * 从 NVS 读取最多五组 Wi-Fi，扫描并连接当前可用网络。
 *
 * 30 秒内无法连接时会启动 SoftAP 配网页面，并停留在配网服务
 * 循环中；用户保存新凭据后设备自动重启。仅在连接成功，或者
 * SoftAP 启动失败时返回。
 */
bool wifiServiceConnect();


/*
 * 清除 NVS 中保存的全部 Wi-Fi 配置。
 *
 * 预留给后续 SYSTEM/WIFI_RESET 协议命令使用。
 */
bool wifiServiceClearProfiles();
