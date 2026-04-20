#ifndef CTRL_PID_H
#define CTRL_PID_H

/*
 * PID 控制器数据结构
 * 说明：
 * - kp/ki/kd：比例、积分、微分系数。
 * - setpoint：目标值。
 * - integral/prev_error：运行时状态量。
 * - integral_limit：积分限幅，防止积分饱和。
 * - out_min/out_max：输出限幅范围。
 */

typedef struct {
    float kp;
    float ki;
    float kd;

    float setpoint;
    float integral;
    float prev_error;

    float integral_limit;
    float out_min;
    float out_max;
} ctrl_pid_t;

// 初始化 PID 参数与初始状态。
void ctrl_pid_init(ctrl_pid_t *pid, float kp, float ki, float kd, float setpoint);

// 更新目标值。
void ctrl_pid_set_setpoint(ctrl_pid_t *pid, float setpoint);

// 更新 PID 三参数。
void ctrl_pid_set_gains(ctrl_pid_t *pid, float kp, float ki, float kd);

// 设置积分限幅绝对值。
void ctrl_pid_set_integral_limit(ctrl_pid_t *pid, float integral_limit);

// 清空积分项和历史误差。
void ctrl_pid_reset(ctrl_pid_t *pid);

// 根据当前测量值和采样周期计算控制输出。
float ctrl_pid_update(ctrl_pid_t *pid, float measurement, float dt_s, float *out_error);

#endif  // CTRL_PID_H
