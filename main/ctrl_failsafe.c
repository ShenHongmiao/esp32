#include "ctrl_failsafe.h"

#include <stddef.h>

void ctrl_failsafe_init(ctrl_failsafe_t *failsafe, uint32_t timeout_ms, float safe_setpoint_c) {
    // 参数保护，避免空指针导致异常。
    if (failsafe == NULL) {
        return;
    }

    failsafe->heartbeat_timeout_ms = timeout_ms;
    failsafe->safe_setpoint_c = safe_setpoint_c;
    failsafe->safe_mode = false;
}

float ctrl_failsafe_effective_setpoint(
    ctrl_failsafe_t *failsafe,
    uint32_t now_ms,
    uint32_t last_heartbeat_ms,
    float requested_setpoint_c) {
    // 未启用或未初始化时，直接使用请求设定值。
    if (failsafe == NULL) {
        return requested_setpoint_c;
    }

    // 基于无符号时间差计算心跳间隔，兼容计数回绕场景。
    const uint32_t delta_ms = now_ms - last_heartbeat_ms;
    if (delta_ms > failsafe->heartbeat_timeout_ms) {
        // 超时进入安全模式。
        failsafe->safe_mode = true;
        return failsafe->safe_setpoint_c;
    }

    // 心跳正常，退出安全模式并沿用请求值。
    failsafe->safe_mode = false;
    return requested_setpoint_c;
}
