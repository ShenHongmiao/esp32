#ifndef COMM_UDP_H
#define COMM_UDP_H

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

/*
 * UDP 通讯模块
 * 功能：
 * 1) 启动 WiFi STA 并管理连接状态
 * 2) 创建 UDP socket 并发送遥测帧
 * 3) 接收上位机文本命令
 */

// 启动 WiFi 与 UDP socket。
esp_err_t comm_udp_start(void);

// 返回当前 WiFi 是否已连接。
bool comm_udp_is_connected(void);

// 发送一帧 UDP 数据到配置的远端地址。
esp_err_t comm_udp_send(const uint8_t *data, size_t len);

// 接收一行文本命令；超时返回 0，错误返回负值。
int comm_udp_receive_line(char *out_line, size_t out_len, int timeout_ms);

#endif  // COMM_UDP_H
