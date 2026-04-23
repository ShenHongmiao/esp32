#include "periph_adc.h"

#include "app_config.h"
#include "periph_i2c.h"

esp_err_t periph_adc_read_raw12(uint8_t command, uint16_t *raw12) {
    // 输出指针检查，避免写野指针。
    if (raw12 == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 线程安全由 periph_i2c_write_then_read 内部的 I2C 总线互斥锁保证。
    // 芯片返回两个字节：高字节低 4 位 + 低字节 8 位，共 12 位。
    uint8_t rx[2] = {0};
    esp_err_t err = periph_i2c_write_then_read(APP_EXT_ADC_ADDR, &command, 1, rx, sizeof(rx));
    if (err != ESP_OK) {
        return err;
    }

    // 按协议拼接：D11..D8 来自 rx[0] 低 4 位，D7..D0 来自 rx[1]。
    *raw12 = (uint16_t)(((rx[0] & 0x0F) << 8) | rx[1]);
    return ESP_OK;
}

float periph_adc_raw12_to_voltage(uint16_t raw12, float vref_v) {
    // 对异常值做上限夹紧，防止比例换算溢出。
    if (raw12 > (uint16_t)APP_ADC_MAX_RAW) {
        raw12 = (uint16_t)APP_ADC_MAX_RAW;
    }

    // 线性换算：V = raw / 4095 * Vref。
    return ((float)raw12 / APP_ADC_MAX_RAW) * vref_v;
}

float periph_adc_calc_supply_voltage(uint16_t raw12) {
    // 先得到分压点电压，再按分压比恢复输入总电压。
    const float v_detect = periph_adc_raw12_to_voltage(raw12, APP_ADC_VREF_V);
    const float ratio = (APP_VDET_R_TOP_OHM + APP_VDET_R_BOTTOM_OHM) / APP_VDET_R_BOTTOM_OHM;
    return v_detect * ratio;
}
