#ifndef PERIPH_PRESSURE_DC_H
#define PERIPH_PRESSURE_DC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef struct {
    float pressure_kpa;
    float voltage_v;
} periph_pressure_dc_sample_t;

// 读取指定 ADC 命令对应的 DC 压力通道，并完成传感器电压与 kPa 换算。
esp_err_t periph_pressure_dc_read_channel(uint8_t adc_cmd, periph_pressure_dc_sample_t *out_sample);

// Convert sensor output voltage to kPa using Vout = slope * P + offset.
float periph_pressure_dc_voltage_to_kpa(float voltage_v);

#endif  // PERIPH_PRESSURE_DC_H
