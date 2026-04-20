#ifndef PERIPH_WF5803F_H
#define PERIPH_WF5803F_H

#include "esp_err.h"

/*
 * WF5803F 压力/温度传感器驱动接口
 * 功能：触发一次转换，等待完成后读取并换算温度与压力。
 */

// 读取一次温度和压力，成功时输出摄氏度与 kPa。
esp_err_t periph_wf5803f_read(float *temperature_c, float *pressure_kpa);

#endif  // PERIPH_WF5803F_H
