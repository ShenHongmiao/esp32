#include "ctrl_pid.h"

#include <math.h>
#include <stddef.h>

#include "app_config.h"

#ifndef APP_PID_DEADBAND_C
#define APP_PID_DEADBAND_C 0.0f
#endif

#ifndef APP_PID_ENABLE_INTEGRAL_SEPARATION
#define APP_PID_ENABLE_INTEGRAL_SEPARATION 0
#endif

#ifndef APP_PID_INTEGRAL_SEPARATION_THRESHOLD_C
#define APP_PID_INTEGRAL_SEPARATION_THRESHOLD_C 0.0f
#endif

#ifndef APP_PID_OUTPUT_MIN_MS
#define APP_PID_OUTPUT_MIN_MS 0.0f
#endif

#ifndef APP_PID_OUTPUT_MAX_MS
#define APP_PID_OUTPUT_MAX_MS 1000.0f
#endif

// 通用限幅函数，供积分项与输出项复用。
static float clamp(float value, float min_v, float max_v) {
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

void ctrl_pid_init(ctrl_pid_t *pid, float kp, float ki, float kd, float setpoint) {
    // 允许上层在异常情况下传空指针，函数内部直接忽略。
    if (pid == NULL) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->setpoint = setpoint;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;

    pid->integral_limit = 100.0f;
    pid->out_min = APP_PID_OUTPUT_MIN_MS;
    pid->out_max = APP_PID_OUTPUT_MAX_MS;
}

void ctrl_pid_set_setpoint(ctrl_pid_t *pid, float setpoint) {
    if (pid == NULL) {
        return;
    }
    pid->setpoint = setpoint;
}
//ctrl_pid_set_gains用于同时设置 PID 的三个增益参数，简化外部调用。
void ctrl_pid_set_gains(ctrl_pid_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
    }

    if (pid->ki != ki) {
        // 与参考实现一致：改 Ki 时清积分，避免历史积分与新参数冲突。
        pid->integral = 0.0f;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void ctrl_pid_set_integral_limit(ctrl_pid_t *pid, float integral_limit) {
    if (pid == NULL) {
        return;
    }

    // 积分限幅始终按绝对值生效。
    if (integral_limit < 0.0f) {
        integral_limit = -integral_limit;
    }
    pid->integral_limit = integral_limit;
}

void ctrl_pid_reset(ctrl_pid_t *pid) {
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float ctrl_pid_update(ctrl_pid_t *pid, float measurement, float dt_s, float *out_error) {
    // dt_s 必须为正数，否则微分项会产生除零风险。
    if (pid == NULL || dt_s <= 0.0f) {
        if (out_error != NULL) {
            *out_error = 0.0f;
        }
        return 0.0f;
    }

    // 误差定义：目标 - 测量。
    const float error = pid->setpoint - measurement;

    // 死区内保持当前输出，且不继续积分。
    if (fabsf(error) < APP_PID_DEADBAND_C) {
        if (out_error != NULL) {
            *out_error = error;
        }
        return pid->output;
    }

    // 积分项：可选积分分离，避免大偏差时积分饱和。
#if APP_PID_ENABLE_INTEGRAL_SEPARATION
    if (fabsf(error) <= APP_PID_INTEGRAL_SEPARATION_THRESHOLD_C) {
        pid->integral += error * dt_s;
        pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
    }
#else
    pid->integral += error * dt_s;
    pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);
#endif

    // 微分项按离散差分计算。
    const float derivative = (error - pid->prev_error) / dt_s;

    // PID 三项叠加并记录输出状态。
    pid->output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
    pid->output = clamp(pid->output, pid->out_min, pid->out_max);

    pid->prev_error = error;

    if (out_error != NULL) {
        *out_error = error;
    }

    return pid->output;
}
