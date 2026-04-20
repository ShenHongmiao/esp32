#ifndef CTRL_NTC_H
#define CTRL_NTC_H

#include <stdbool.h>

/*
 * NTC 温度换算模块
 * 输入：分压采样电压
 * 输出：摄氏温度
 * 公式：Beta 模型
 */

// 将 NTC 分压电压换算为温度（℃）；返回 false 代表输入无效或计算失败。
bool ctrl_ntc_voltage_to_temp_c(float voltage_v, float *out_temp_c);

#endif  // CTRL_NTC_H
