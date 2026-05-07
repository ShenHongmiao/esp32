#ifndef PERIPH_PWM_H
#define PERIPH_PWM_H

#include <stdint.h>

#include "esp_err.h"

/*
 * PWM 输出抽象层
 * - 支持双通道独立控制，也可同时写入两路。
 * - 统一由控制层调用，避免应用层直接操作 LEDC 细节。
 */

// 初始化 LEDC 定时器与通道，默认占空比为 0%。
esp_err_t periph_pwm_init(void);

// 设置指定通道的 1s 窗口内导通时长（0~1000ms）。
void periph_pwm_set_on_time_ms_ch(uint8_t channel, float on_time_ms);

// 设置 1s 窗口内导通时长（0~1000ms），同时作用于两路通道。
void periph_pwm_set_on_time_ms(float on_time_ms);

// 设置占空比百分比（0~100）。
void periph_pwm_set_percent(float duty_percent);

// 强制关闭指定通道输出（占空比置 0）。
void periph_pwm_force_off_ch(uint8_t channel);

// 强制关闭输出（占空比置 0），作用于两路通道。
void periph_pwm_force_off(void);

// 获取最近一次设置的导通时长（ms）。
float periph_pwm_get_on_time_ms(void);

// 获取最近一次设置的占空比百分比。
float periph_pwm_get_percent(void);

#endif  // PERIPH_PWM_H
