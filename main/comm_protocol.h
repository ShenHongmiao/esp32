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
    COMM_CMD_PID_OUT = 0x04,
    COMM_CMD_PRESSURE = 0x05,
    COMM_CMD_TEXT_INFO = 0x0F,
};

// 计算 CRC8（多项式 0x07）。
uint8_t comm_protocol_crc8(const uint8_t *data, size_t len);

// 组装完整数据帧，返回实际帧长度；失败返回 0。
size_t comm_protocol_build_frame(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_frame, size_t out_cap);

// 打包 NTC 温度载荷（按开关决定是否写入对应通道）。
// 注意：该函数原用于打包 NTC0/1 物理通道数据，现已重构为打包 PWM CH0 和 PWM CH1 的控制反馈平均温度。
// 入参中的 ch0_xxx 对应 PWM CH0 (NTC2/3)，ch1_xxx 对应 PWM CH1 (NTC0/1)。
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

// 打包 PID 输出载荷（单位：ms），单路输出。
size_t comm_protocol_pack_pid_out_payload(float pid_out_ms, uint8_t *out_payload, size_t out_cap);

/**
 * @brief 动态打包压力通道的 Payload (CMD 0x05)
 * @param payload_buf 用于存放打包后载荷的缓冲区指针
 * @param mask 通道掩码 (bit0: CH1使能, bit1: CH2使能)
 * @param ch1_val CH1 的压力值 (kPa)
 * @param ch2_val CH2 的压力值 (kPa)
 * @return 组装后的有效载荷实际长度 (字节数)
 */
size_t comm_protocol_pack_dynamic_pressure_payload(uint8_t *payload_buf, uint8_t mask, float ch1_val, float ch2_val);

// 打包文本信息载荷。
size_t comm_protocol_pack_text_payload(const char *text, uint8_t *out_payload, size_t out_cap);

#endif  // COMM_PROTOCOL_H
