#include "periph_wf5803f.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_config.h"
#include "periph_i2c.h"

// WF5803F 关键寄存器定义。
#define WF5803F_REG_STATUS       0x02
#define WF5803F_REG_PRESS_MSB    0x06
#define WF5803F_REG_CMD          0x30
#define WF5803F_CMD_SINGLE_TP    0x0A

static float wf5803f_pressure_from_raw24(int32_t raw) {
    // 仅保留 24 位有效数据。
    raw &= 0x00FFFFFF;

    // 若 bit23 为 1，执行符号扩展到 32 位。
    if (raw & 0x00800000) {
        raw |= 0xFF000000;
    }

    // 归一化并代入规格书给定的 2bar 型号换算公式（输出 kPa）。
    const float factor = (float)raw / 8388608.0f;
    return 180.0f / 0.81f * (factor - 0.1f) + 30.0f;
}

static float wf5803f_temp_from_raw16(int16_t raw) {
    // 温度分辨率为 1/256 ℃。
    return (float)raw / 256.0f;
}

esp_err_t periph_wf5803f_read(float *temperature_c, float *pressure_kpa) {
    // 参数检查：两个输出都必须可写。
    if (temperature_c == NULL || pressure_kpa == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 写命令寄存器，启动单次温压联合测量。
    esp_err_t err = periph_i2c_write_reg(APP_WF5803F_USE_ADDR, WF5803F_REG_CMD, WF5803F_CMD_SINGLE_TP);
    if (err != ESP_OK) {
        return err;
    }

    // 轮询状态寄存器 bit0，等待转换完成。
    for (int i = 0; i < 20; ++i) {
        uint8_t status = 0;
        err = periph_i2c_read_reg(APP_WF5803F_USE_ADDR, WF5803F_REG_STATUS, &status);
        if (err != ESP_OK) {
            return err;
        }
        if (status & 0x01) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
        if (i == 19) {
            return ESP_ERR_TIMEOUT;
        }
    }

    // 从 0x06 连续读取 5 字节：压力 3 字节 + 温度 2 字节。
    uint8_t reg = WF5803F_REG_PRESS_MSB;
    uint8_t data[5] = {0};
    err = periph_i2c_write_then_read(APP_WF5803F_USE_ADDR, &reg, 1, data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    // 按大端拼接传感器原始数据。
    int32_t press_raw = ((int32_t)data[0] << 16) | ((int32_t)data[1] << 8) | data[2];
    int16_t temp_raw = (int16_t)(((uint16_t)data[3] << 8) | data[4]);

    // 原始值换算为工程单位。
    *pressure_kpa = wf5803f_pressure_from_raw24(press_raw);
    *temperature_c = wf5803f_temp_from_raw16(temp_raw);
    return ESP_OK;
}
