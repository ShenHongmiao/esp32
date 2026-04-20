#ifndef PERIPH_ADC_H
#define PERIPH_ADC_H

#include <stdint.h>

#include "esp_err.h"

/*
 * 外部 ADC 抽象层
 * - 负责按命令读取 12bit 原始值
 * - 提供“原始值 -> 电压”与“分压电压 -> 实际输入电压”的工具函数
 */

// 按命令字读取一个 12bit 原始采样值。
esp_err_t periph_adc_read_raw12(uint8_t command, uint16_t *raw12);

// 将 12bit 原始值按参考电压换算为电压值。
float periph_adc_raw12_to_voltage(uint16_t raw12, float vref_v);

// 将 V_DETECT 通道电压根据分压比换算为输入侧实际电压。
float periph_adc_calc_supply_voltage(uint16_t raw12);

#endif  // PERIPH_ADC_H
