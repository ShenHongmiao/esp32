#include "comm_protocol.h"

#include <string.h>

// 写入 16 位小端整数。
static void write_i16_le(uint8_t *buf, int16_t value) {
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
}

// 写入 32 位小端整数。
static void write_i32_le(uint8_t *buf, int32_t value) {
    buf[0] = (uint8_t)(value & 0xFF);
    buf[1] = (uint8_t)((value >> 8) & 0xFF);
    buf[2] = (uint8_t)((value >> 16) & 0xFF);
    buf[3] = (uint8_t)((value >> 24) & 0xFF);
}

// 浮点值放大 100 倍后压缩为 int16，包含上下限保护。
static int16_t scale100_to_i16(float value) {
    const float scaled = value * 100.0f;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return -32768;
    }
    return (int16_t)scaled;
}

// 浮点值放大 100 倍后压缩为 int32，包含上下限保护。
static int32_t scale100_to_i32(float value) {
    const float scaled = value * 100.0f;
    if (scaled > 2147483647.0f) {
        return 2147483647;
    }
    if (scaled < -2147483648.0f) {
        return (int32_t)0x80000000;
    }
    return (int32_t)scaled;
}

uint8_t comm_protocol_crc8(const uint8_t *data, size_t len) {
    // 初值 0x00，逐字节异或并按 0x07 多项式移位。
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t comm_protocol_build_frame(uint8_t cmd_id, const uint8_t *payload, uint8_t payload_len, uint8_t *out_frame, size_t out_cap) {
    // 输出缓存不能为空。
    if (out_frame == NULL) {
        return 0;
    }

    // 固定开销：头 1 + cmd 1 + len 1 + crc 1 + 尾 1。
    const size_t frame_len = 1u + 1u + 1u + payload_len + 1u + 1u;
    if (out_cap < frame_len) {
        return 0;
    }

    out_frame[0] = COMM_FRAME_HEAD;
    out_frame[1] = cmd_id;
    out_frame[2] = payload_len;

    if (payload_len > 0 && payload != NULL) {
        memcpy(&out_frame[3], payload, payload_len);
    }

    // CRC 覆盖范围：从帧头到 payload 末尾。
    out_frame[3 + payload_len] = comm_protocol_crc8(out_frame, 3 + payload_len);
    out_frame[4 + payload_len] = COMM_FRAME_TAIL;
    return frame_len;
}

size_t comm_protocol_pack_ntc_payload(
    bool ch0_enable,
    float ch0_temp_c,
    bool ch1_enable,
    float ch1_temp_c,
    uint8_t *out_payload,
    size_t out_cap) {
    // 按通道使能动态拼接，保证与需求中的可选字段一致。
    if (out_payload == NULL) {
        return 0;
    }

    size_t index = 0;
    if (ch0_enable) {
        if (index + 2 > out_cap) {
            return 0;
        }
        write_i16_le(&out_payload[index], scale100_to_i16(ch0_temp_c));
        index += 2;
    }

    if (ch1_enable) {
        if (index + 2 > out_cap) {
            return 0;
        }
        write_i16_le(&out_payload[index], scale100_to_i16(ch1_temp_c));
        index += 2;
    }

    return index;
}

size_t comm_protocol_pack_wf_payload(float temperature_c, float pressure_kpa, uint8_t *out_payload, size_t out_cap) {
    // 固定字段：int16 温度 + int32 压力，共 6 字节。
    if (out_payload == NULL || out_cap < 6) {
        return 0;
    }

    write_i16_le(&out_payload[0], scale100_to_i16(temperature_c));
    write_i32_le(&out_payload[2], scale100_to_i32(pressure_kpa));
    return 6;
}

size_t comm_protocol_pack_voltage_payload(float voltage_v, bool undervoltage, uint8_t *out_payload, size_t out_cap) {
    // 固定字段：int16 电压 + uint8 状态，共 3 字节。
    if (out_payload == NULL || out_cap < 3) {
        return 0;
    }

    write_i16_le(&out_payload[0], scale100_to_i16(voltage_v));
    out_payload[2] = undervoltage ? 0xFF : 0x01;
    return 3;
}

size_t comm_protocol_pack_text_payload(const char *text, uint8_t *out_payload, size_t out_cap) {
    // 文本按原样拷贝，不做编码转换。
    if (text == NULL || out_payload == NULL || out_cap == 0) {
        return 0;
    }

    const size_t len = strnlen(text, out_cap);
    memcpy(out_payload, text, len);
    return len;
}
