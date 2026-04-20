#ifndef CTRL_FAILSAFE_H
#define CTRL_FAILSAFE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 失联保护模块
 * 场景：上位机心跳丢失超过阈值时，将控制目标切换到安全温度。
 */

typedef struct {
    // 允许的最大心跳间隔。
    uint32_t heartbeat_timeout_ms;
    // 触发保护后使用的安全设定温度。
    float safe_setpoint_c;
    // 当前是否处于安全模式（由模块内部维护）。
    bool safe_mode;
} ctrl_failsafe_t;

// 初始化失联保护参数。
void ctrl_failsafe_init(ctrl_failsafe_t *failsafe, uint32_t timeout_ms, float safe_setpoint_c);

// 根据当前时间与最后心跳时间，给出实际应使用的设定温度。
float ctrl_failsafe_effective_setpoint(
    ctrl_failsafe_t *failsafe,
    uint32_t now_ms,
    uint32_t last_heartbeat_ms,
    float requested_setpoint_c);

#endif  // CTRL_FAILSAFE_H
