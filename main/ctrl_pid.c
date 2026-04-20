#include "ctrl_pid.h"

#include <stddef.h>

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

    pid->integral_limit = 100.0f;
    pid->out_min = 0.0f;
    pid->out_max = 100.0f;
}

void ctrl_pid_set_setpoint(ctrl_pid_t *pid, float setpoint) {
    if (pid == NULL) {
        return;
    }
    pid->setpoint = setpoint;
}

void ctrl_pid_set_gains(ctrl_pid_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) {
        return;
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

    // 积分累加并限幅，抑制积分饱和。
    pid->integral += error * dt_s;
    pid->integral = clamp(pid->integral, -pid->integral_limit, pid->integral_limit);

    // 微分项按离散差分计算。
    const float derivative = (error - pid->prev_error) / dt_s;

    // PID 三项叠加。
    const float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    pid->prev_error = error;

    if (out_error != NULL) {
        *out_error = error;
    }

    // 最终输出再进行一次边界保护。
    return clamp(output, pid->out_min, pid->out_max);
}
