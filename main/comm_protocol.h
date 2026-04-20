#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 通讯协议封装模块
 * 帧结构：HEAD + CMD + LEN + PAYLOAD + CRC8 + TAIL
 * 约定：
 * - 小端序
 * - 浮点传输前统一放大 100 倍转整数
 */

#define COMM_FRAME_HEAD  0xDE
#define COMM_FRAME_TAIL  0xED

enum {
    COMM_CMD_NTC = 0x01,
    COMM_CMD_WF5803F = 0x02,
    COMM_CMD_VOLTAGE = 0x03,
    COMM_CMD_TEXT_INFO = 0x0F,
};

// 计算 CRC8（多项式 0x07）。
uint8_t comm_protocol_crc8(const uint8_t *data, size_t len);

// 组装完整数据帧，返回实际帧长度；失败返回 0。
size_t comm_protocol_build_frame(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_frame, size_t out_cap);

// 打包 NTC 温度载荷（按开关决定是否写入对应通道）。
size_t comm_protocol_pack_ntc_payload(
    bool ch0_enable,
    float ch0_temp_c,
    bool ch1_enable,
    float ch1_temp_c,
    uint8_t *out_payload,
    size_t out_cap);

// 打包 WF5803F 温度与压力载荷。
size_t comm_protocol_pack_wf_payload(float temperature_c, float pressure_kpa, uint8_t *out_payload, size_t out_cap);

// 打包系统电压载荷。
size_t comm_protocol_pack_voltage_payload(float voltage_v, bool undervoltage, uint8_t *out_payload, size_t out_cap);

// 打包文本信息载荷。
size_t comm_protocol_pack_text_payload(const char *text, uint8_t *out_payload, size_t out_cap);

#endif  // COMM_PROTOCOL_H
