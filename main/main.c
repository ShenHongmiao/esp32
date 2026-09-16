#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "app_config.h"
#include "pressure_config.h"
#include "comm_command.h"
#include "comm_protocol.h"
#include "comm_udp.h"
#include "ctrl_failsafe.h"
#include "ctrl_ntc.h"
#include "ctrl_pid.h"
#include "periph_adc.h"
#include "periph_i2c.h"
#include "periph_pressure_dc.h"
#include "periph_pwm.h"
#include "periph_wf5803f.h"
#include "sys_ota.h"

/*
 * 主应用编排层
 * 设计目标：
 * 1) 外设采样、控制算法、通讯、OTA 分任务解耦。
 * 2) 共享状态通过互斥锁保护，避免多任务并发读写冲突。
 * 3) 所有参数从 app_config.h 读取，便于集中管理。
 */

typedef struct {
    // NTC 四路温度及其有效标志（对应 CH1-CH4）。
	float ntc_temp_c[4];
	float ntc_voltage_v[4];
	bool ntc_valid[4];

    // WF5803F 温度/压力及有效标志。
	float wf_temp_c;
	float wf_pressure_kpa;
	bool wf_valid;

	// DC pressure sensor sample (voltage output, dual channel).
	float dc_pressure_kpa_ch1;
	float dc_pressure_kpa_ch2;
	uint8_t pressure_mask; // bit 0 for CH1, bit 1 for CH2

    // 电源电压与欠压状态。
	float supply_voltage_v;
	bool undervoltage;

    // 控制相关运行态（双路 PID）。
	float process_temp_c[2];
	float requested_setpoint_c;
	float effective_setpoint_c[2];
	float pwm_on_ms[2];
	// 模式 4 的基准加热时长仅保存在 RAM，上电时总是恢复宏默认值。
	float mode4_heat_time_base_ms;
	// SP/HTIME 命令每到达一次都递增版本号，即使新值与旧值相同也能触发状态机复位。
	uint32_t mode4_config_revision;

    // 上位机心跳时间戳与 OTA 请求标志。
	uint32_t last_heartbeat_ms;
	bool ota_pending;
} app_runtime_t;

static const char *TAG = "heater_app";

#define APP_CONTROL_GROUPS 2

typedef struct {
	uint8_t primary_ntc;
	uint8_t secondary_ntc;
	uint8_t pwm_channel;
} control_group_map_t;

static const control_group_map_t s_group_map[APP_CONTROL_GROUPS] = {
	{0, 1, 1},
	{2, 3, 0},
};

typedef struct {
	uint8_t adc_cmd;
	bool enabled;
} ntc_sample_config_t;

// 四路 NTC 的 ADC 命令和编译期使能集中在同一张表中。
// 采样任务只遍历此表，避免四段结构相同的条件编译代码逐渐失配。
static const ntc_sample_config_t s_ntc_sample_config[4] = {
	{APP_EXT_ADC_CMD_NTC0, FEATURE_NTC_CH0_ENABLE != 0},
	{APP_EXT_ADC_CMD_NTC1, FEATURE_NTC_CH1_ENABLE != 0},
	{APP_EXT_ADC_CMD_NTC2, FEATURE_NTC_CH2_ENABLE != 0},
	{APP_EXT_ADC_CMD_NTC3, FEATURE_NTC_CH3_ENABLE != 0},
};

#if APP_NTC_FILTER_WINDOW_SIZE < 1
#error "APP_NTC_FILTER_WINDOW_SIZE must be >= 1"
#endif

#if APP_NTC_SAMPLE_PERIOD_MS < 1
#error "APP_NTC_SAMPLE_PERIOD_MS must be >= 1"
#endif

typedef struct {
	float voltage_ring[4][APP_NTC_FILTER_WINDOW_SIZE];
	float voltage_sum[4];
	float filtered_voltage_v[4];
	uint8_t ring_head[4];
	uint8_t sample_count[4];
	bool filtered_valid[4];
} ntc_filter_state_t;

static SemaphoreHandle_t s_state_lock;
static app_runtime_t s_state;
static ctrl_pid_t s_pid[APP_CONTROL_GROUPS];
static ctrl_failsafe_t s_failsafe;
static ntc_filter_state_t s_ntc_filter;
#if FEATURE_HEATING_MODE == 2
static uint8_t s_cyclic_stage[APP_CONTROL_GROUPS] = {1, 1};
static uint32_t s_cyclic_hold_start_ms[APP_CONTROL_GROUPS];
static bool s_cyclic_hold_active[APP_CONTROL_GROUPS];
#endif
static float s_last_target_sp[APP_CONTROL_GROUPS] = {APP_DEFAULT_SETPOINT_C, APP_DEFAULT_SETPOINT_C};

#if FEATURE_HEATING_MODE == 3
// 模式 3 状态机定义：双通道互锁交替。
typedef enum {
    MODE3_CH0_HEAT,   // CH0 加热（目标高温），CH1 冷却（目标低温）
    MODE3_CH0_COOL,   // CH0 已到高温，等待 CH0 降温到触发点
    MODE3_CH1_HEAT,   // CH1 加热（目标高温），CH0 冷却（目标低温）
    MODE3_CH1_COOL,   // CH1 已到高温，等待 CH1 降温到触发点
} mode3_state_t;
static mode3_state_t s_mode3_state = MODE3_CH0_HEAT;
#endif

#if FEATURE_HEATING_MODE == 4
// 模式 4 每个控制组都使用一套完全独立的三态状态机，两路之间不互锁。
typedef enum {
	MODE4_STABILIZE = 0, // 用 PID 将过程温度稳定在 T_low。
	MODE4_HEAT,          // 输出 1000ms（即 100% 占空比）的定长脉冲。
	MODE4_COOL,          // 强制输出 0ms，捕获热惯性峰值并等待降温。
} mode4_state_id_t;

typedef struct {
	mode4_state_id_t state;
	uint32_t state_enter_ms;
	uint32_t stable_start_ms;
	bool stable_timer_active;
	uint32_t applied_config_revision;
	float configured_low_c;
	float heat_time_base_ms;
	float heat_time_ms;
	float heat_start_temp_c;
	float heat_end_temp_c;
	float peak_temp_c;
	bool converged;
	bool invalid_range_warned;
	bool saturation_warned;
} mode4_group_state_t;

static mode4_group_state_t s_mode4[APP_CONTROL_GROUPS];

// 将需求文档中与 20ms 控制分辨率直接相关的硬约束变成编译期检查，
// 防止日后调参时将峰值容差设得过小，导致脉冲在相邻周期之间往复振荡。
_Static_assert(APP_CONTROL_PERIOD_MS > 0, "mode4 requires a positive control period");
_Static_assert(APP_MODE4_PEAK_TOL_C >= 1.0f, "APP_MODE4_PEAK_TOL_C must be >= 1.0C");
_Static_assert(APP_MODE4_HEAT_TIME_MIN_MS > 0.0f,
			   "APP_MODE4_HEAT_TIME_MIN_MS must be positive");
_Static_assert(APP_MODE4_HEAT_TIME_DEFAULT_MS >= APP_MODE4_HEAT_TIME_MIN_MS &&
			   APP_MODE4_HEAT_TIME_DEFAULT_MS <= APP_MODE4_HEAT_TIME_MAX_MS,
			   "mode4 default heat time must be inside min/max bounds");
_Static_assert(APP_MODE4_HEAT_TIMEOUT_FACTOR >= 1.0f,
			   "mode4 timeout factor must not be shorter than the requested pulse");
#endif

static uint32_t app_now_ms(void) {
	// 统一使用 esp_timer 提供的微秒计时，再转换为毫秒。
	return (uint32_t)(esp_timer_get_time() / 1000ULL);
}
static esp_err_t init_nvs(void) {
	// NVS 初始化失败且提示页满/版本不一致时，先擦除再重建。
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	return err;
}

static void runtime_init(void) {
	// 初始化关键状态，确保控制器启动时有确定行为。
	s_state.requested_setpoint_c = APP_DEFAULT_SETPOINT_C;
	for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
		s_state.effective_setpoint_c[group] = APP_DEFAULT_SETPOINT_C;
		s_state.process_temp_c[group] = APP_DEFAULT_SETPOINT_C;
		s_state.pwm_on_ms[group] = 0.0f;
		s_last_target_sp[group] = APP_DEFAULT_SETPOINT_C;
	}
	s_state.last_heartbeat_ms = app_now_ms();
	s_state.mode4_heat_time_base_ms = APP_MODE4_HEAT_TIME_DEFAULT_MS;
	s_state.mode4_config_revision = 0U;
	s_state.dc_pressure_kpa_ch1 = 0.0f;
	s_state.dc_pressure_kpa_ch2 = 0.0f;
	s_state.pressure_mask = 0;
}

static bool get_group_average_temperature(const app_runtime_t *sample,
										 uint32_t group,
										 float *out_temp_c) {
	// 控制与遥测共用同一个分组求平均实现，分组关系只从 s_group_map 读取。
	// 任一传感器有效时即可返回该组温度；两个都有效时取算术平均。
	if (sample == NULL || out_temp_c == NULL || group >= APP_CONTROL_GROUPS) {
		return false;
	}

	const uint8_t channels[2] = {
		s_group_map[group].primary_ntc,
		s_group_map[group].secondary_ntc,
	};
	float sum = 0.0f;
	uint32_t valid_count = 0U;
	for (uint32_t i = 0; i < 2U; ++i) {
		const uint8_t channel = channels[i];
		if (sample->ntc_valid[channel]) {
			sum += sample->ntc_temp_c[channel];
			valid_count++;
		}
	}

	if (valid_count == 0U) {
		*out_temp_c = NAN;
		return false;
	}

	*out_temp_c = sum / (float)valid_count;
	return true;
}

static void ntc_filter_reset(ntc_filter_state_t *filter) {
	if (filter == NULL) {
		return;
	}

	for (uint32_t ch = 0; ch < 4; ++ch) {
		filter->voltage_sum[ch] = 0.0f;
		filter->filtered_voltage_v[ch] = 0.0f;
		filter->ring_head[ch] = 0;
		filter->sample_count[ch] = 0;
		filter->filtered_valid[ch] = false;
		for (uint32_t i = 0; i < APP_NTC_FILTER_WINDOW_SIZE; ++i) {
			filter->voltage_ring[ch][i] = 0.0f;
		}
	}
}

#if FEATURE_HEATING_MODE == 2
static float update_cyclic_setpoint_group(uint32_t group,
									float stage1_setpoint,
									float process_temp,
									bool process_valid) {
	const uint32_t now_ms = app_now_ms();
	float target = (s_cyclic_stage[group] == 2U) ? APP_CYCLIC_SETPOINT2_C : stage1_setpoint;
	bool stable = process_valid;

	if (stable) {
		if (fabsf(process_temp - target) > APP_CYCLIC_HOLD_THRESHOLD_C) {
			stable = false;
		}
	}

	if (APP_CYCLIC_HOLD_TIME_MS == 0U) {
		stable = false;
	}

	if (stable) {
		if (!s_cyclic_hold_active[group]) {
			s_cyclic_hold_start_ms[group] = now_ms;
			s_cyclic_hold_active[group] = true;
		} else if ((now_ms - s_cyclic_hold_start_ms[group]) >= APP_CYCLIC_HOLD_TIME_MS) {
			s_cyclic_stage[group] = (s_cyclic_stage[group] == 1U) ? 2U : 1U;
			s_cyclic_hold_active[group] = false;
			target = (s_cyclic_stage[group] == 2U) ? APP_CYCLIC_SETPOINT2_C : stage1_setpoint;
		}
	} else {
		s_cyclic_hold_active[group] = false;
	}

	return target;
}
#endif

static bool sample_ntc_voltage_channel(uint8_t channel, uint8_t adc_cmd, float *out_voltage_v) {
	uint16_t raw = 0;
	const esp_err_t err = periph_adc_read_raw12(adc_cmd, &raw);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "ntc ch%u adc read failed: %s", channel, esp_err_to_name(err));
		return false;
	}

	*out_voltage_v = periph_adc_raw12_to_voltage(raw, APP_ADC_VREF_V);
	return true;
}

static void ntc_filter_push_voltage_locked(ntc_filter_state_t *filter, uint8_t channel, float voltage_v) {
	const uint8_t head = filter->ring_head[channel];
	uint8_t count = filter->sample_count[channel];

	if (count == APP_NTC_FILTER_WINDOW_SIZE) {
		filter->voltage_sum[channel] -= filter->voltage_ring[channel][head];
	} else {
		count++;
	}

	filter->voltage_ring[channel][head] = voltage_v;
	filter->voltage_sum[channel] += voltage_v;
	filter->sample_count[channel] = count;
	filter->ring_head[channel] = (uint8_t)((head + 1U) % APP_NTC_FILTER_WINDOW_SIZE);
	filter->filtered_voltage_v[channel] = filter->voltage_sum[channel] / (float)count;
	filter->filtered_valid[channel] = true;
}

static void sample_non_ntc_peripherals(app_runtime_t *sample) {
	sample->wf_valid = false;

#if FEATURE_VOLTAGE_MONITOR_ENABLE
	// 电源检测每个控制周期采样一次。
	uint16_t raw_vdet = 0;
	if (periph_adc_read_raw12(APP_EXT_ADC_CMD_VDETECT, &raw_vdet) == ESP_OK) {
		sample->supply_voltage_v = periph_adc_calc_supply_voltage(raw_vdet);
		sample->undervoltage = sample->supply_voltage_v < APP_UNDERVOLTAGE_THRESHOLD_V;
	}
#endif

#if APP_PRESSURE_SOURCE_DC
	// 压力外设层统一完成 ADC 读取、分压还原、kPa 换算和负值钳位。
	// 本层只遍历启用的逻辑通道并记录有效位，不再复制硬件换算公式。
	static const uint8_t pressure_adc_cmd[2] = {
		APP_EXT_ADC_CMD_Press1,
		APP_EXT_ADC_CMD_Press2,
	};
	static const bool pressure_enabled[2] = {
		APP_PRESSURE_DC_CH1 != 0,
		APP_PRESSURE_DC_CH2 != 0,
	};
	float pressure_kpa[2] = {0.0f, 0.0f};
	sample->pressure_mask = 0U;

	for (uint32_t channel = 0; channel < 2U; ++channel) {
		if (!pressure_enabled[channel]) {
			continue;
		}
		periph_pressure_dc_sample_t pressure_sample = {0};
		if (periph_pressure_dc_read_channel(pressure_adc_cmd[channel], &pressure_sample) == ESP_OK) {
			pressure_kpa[channel] = pressure_sample.pressure_kpa;
			sample->pressure_mask |= (uint8_t)(1U << channel);
		}
	}

	sample->dc_pressure_kpa_ch1 = pressure_kpa[0];
	sample->dc_pressure_kpa_ch2 = pressure_kpa[1];
#endif

#if FEATURE_WF5803F_ENABLE
	// 读取 WF5803F 温度与压力。
	if (periph_wf5803f_read(&sample->wf_temp_c, &sample->wf_pressure_kpa) == ESP_OK) {
		sample->wf_valid = true;
	}
#endif
}

static void sampling_task(void *arg) {
	(void)arg;

	const TickType_t sample_period_ticks =
		(pdMS_TO_TICKS(APP_NTC_SAMPLE_PERIOD_MS) > 0) ? pdMS_TO_TICKS(APP_NTC_SAMPLE_PERIOD_MS) : 1;
	uint32_t slow_sample_div_u32 = APP_CONTROL_PERIOD_MS / APP_NTC_SAMPLE_PERIOD_MS;
	if (slow_sample_div_u32 == 0U) {
		slow_sample_div_u32 = 1U;
	}
	if (slow_sample_div_u32 > 255U) {
		slow_sample_div_u32 = 255U;
	}
	const uint8_t slow_sample_div = (uint8_t)slow_sample_div_u32;
	static uint8_t slow_sample_counter = 0;
	TickType_t sys_tick_count_sample = xTaskGetTickCount();
	app_runtime_t slow_sample = {0};
	sample_non_ntc_peripherals(&slow_sample);

	while (1) {
		float sampled_voltage_v[4] = {0};
		bool sampled_valid[4] = {false};
		app_runtime_t sensor_snapshot = slow_sample;

		for (uint32_t ch = 0; ch < 4U; ++ch) {
			if (s_ntc_sample_config[ch].enabled) {
				sampled_valid[ch] = sample_ntc_voltage_channel(
					(uint8_t)ch,
					s_ntc_sample_config[ch].adc_cmd,
					&sampled_voltage_v[ch]);
			}
		}

		for (uint32_t ch = 0; ch < 4; ++ch) {
			if (sampled_valid[ch]) {
				ntc_filter_push_voltage_locked(&s_ntc_filter, (uint8_t)ch, sampled_voltage_v[ch]);
			}

			sensor_snapshot.ntc_valid[ch] = false;
			sensor_snapshot.ntc_voltage_v[ch] = 0.0f;
			sensor_snapshot.ntc_temp_c[ch] = 0.0f;

			if (s_ntc_filter.filtered_valid[ch]) {
				float temp_c = 0.0f;
				const float filtered_voltage_v = s_ntc_filter.filtered_voltage_v[ch];
				if (ctrl_ntc_voltage_to_temp_c(filtered_voltage_v, &temp_c)) {
					sensor_snapshot.ntc_valid[ch] = true;
					sensor_snapshot.ntc_voltage_v[ch] = filtered_voltage_v;
					sensor_snapshot.ntc_temp_c[ch] = temp_c;
				} else {
					ESP_LOGE(TAG, "ntc ch%u convert failed: v=%.3f", (unsigned int)ch, filtered_voltage_v);
				}
			}
		}

		slow_sample_counter++;
		if (slow_sample_counter >= slow_sample_div) {
			sample_non_ntc_peripherals(&slow_sample);
			sensor_snapshot.wf_temp_c = slow_sample.wf_temp_c;
			sensor_snapshot.wf_pressure_kpa = slow_sample.wf_pressure_kpa;
			sensor_snapshot.wf_valid = slow_sample.wf_valid;
			sensor_snapshot.supply_voltage_v = slow_sample.supply_voltage_v;
			sensor_snapshot.undervoltage = slow_sample.undervoltage;
			sensor_snapshot.dc_pressure_kpa_ch1 = slow_sample.dc_pressure_kpa_ch1;
			sensor_snapshot.dc_pressure_kpa_ch2 = slow_sample.dc_pressure_kpa_ch2;
			sensor_snapshot.pressure_mask = slow_sample.pressure_mask;
			slow_sample_counter = 0;
		}

		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			s_state.ntc_temp_c[ch] = sensor_snapshot.ntc_temp_c[ch];
			s_state.ntc_voltage_v[ch] = sensor_snapshot.ntc_voltage_v[ch];
			s_state.ntc_valid[ch] = sensor_snapshot.ntc_valid[ch];
		}
		s_state.wf_temp_c = sensor_snapshot.wf_temp_c;
		s_state.wf_pressure_kpa = sensor_snapshot.wf_pressure_kpa;
		s_state.wf_valid = sensor_snapshot.wf_valid;
		s_state.supply_voltage_v = sensor_snapshot.supply_voltage_v;
		s_state.undervoltage = sensor_snapshot.undervoltage;
		s_state.dc_pressure_kpa_ch1 = sensor_snapshot.dc_pressure_kpa_ch1;
		s_state.dc_pressure_kpa_ch2 = sensor_snapshot.dc_pressure_kpa_ch2;
		s_state.pressure_mask = sensor_snapshot.pressure_mask;
		xSemaphoreGive(s_state_lock);

		vTaskDelayUntil(&sys_tick_count_sample, sample_period_ticks);
	}
}

static void apply_command(const comm_command_t *cmd) {
	// 命令为空时直接忽略。
	if (cmd == NULL) {
		return;
	}

	bool command_accepted = true;
	bool print_status_log = true;

	// 任何能被解析的命令都先刷新心跳时间。共享状态和 PID 参数的修改都在同一个锁内完成。
	xSemaphoreTake(s_state_lock, portMAX_DELAY);
	s_state.last_heartbeat_ms = app_now_ms();

	// 先处理全局命令。PID 参数命令留给下方的统一通道循环，避免每个 case 复制循环框架。
	switch (cmd->type) {
		case COMM_COMMAND_HEARTBEAT:
			// 心跳可能高频到达，只刷新时间戳，不生成全量 INFO 日志。
			print_status_log = false;
			break;
		case COMM_COMMAND_SETPOINT:
			if (!isfinite(cmd->value)) {
				command_accepted = false;
				break;
			}
			s_state.requested_setpoint_c = cmd->value;
			// 模式 4 规定：收到温度命令就必须丢弃已学习的工作时长并重新稳定。
			// 使用事件版本号而不是仅比较浮点值，以便 SP=50 连续发送两次也能复位。
			s_state.mode4_config_revision++;
			break;
		case COMM_COMMAND_KP:
		case COMM_COMMAND_KI:
		case COMM_COMMAND_KD:
		case COMM_COMMAND_ILIMIT:
			// 任何非有限 PID 参数都会污染计算链，并可能在 PWM/遥测换算时触发非法浮点转整数。
			if (!isfinite(cmd->value)) {
				command_accepted = false;
			}
			break;
		case COMM_COMMAND_HTIME:
			// HTIME 是基准时长而不是当前学习值。拒绝 NaN/无穷大及越界值，
			// 避免非法时长绕过状态机的 20~1000ms 安全边界。
			if (isfinite(cmd->value) &&
				cmd->value >= APP_MODE4_HEAT_TIME_MIN_MS &&
				cmd->value <= APP_MODE4_HEAT_TIME_MAX_MS) {
				s_state.mode4_heat_time_base_ms = cmd->value;
				s_state.mode4_config_revision++;
			} else {
				command_accepted = false;
			}
			break;
		case COMM_COMMAND_OTA:
			s_state.ota_pending = true;
			break;
		default:
			command_accepted = false;
			print_status_log = false;
			break;
	}

	// 所有 PID 参数命令通过这一个循环同步到两个控制组。
	// KI 仍保留“数值变化时清积分”的原有语义，ILIMIT 仍通过专用函数处理绝对值。
	if (command_accepted &&
		(cmd->type == COMM_COMMAND_KP || cmd->type == COMM_COMMAND_KI ||
		 cmd->type == COMM_COMMAND_KD || cmd->type == COMM_COMMAND_ILIMIT)) {
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			switch (cmd->type) {
				case COMM_COMMAND_KP:
					s_pid[group].kp = cmd->value;
					break;
				case COMM_COMMAND_KI:
					if (s_pid[group].ki != cmd->value) {
						s_pid[group].ki = cmd->value;
						s_pid[group].integral = 0.0f;
					}
					break;
				case COMM_COMMAND_KD:
					s_pid[group].kd = cmd->value;
					break;
				case COMM_COMMAND_ILIMIT:
					ctrl_pid_set_integral_limit(&s_pid[group], cmd->value);
					break;
				default:
					break;
			}
		}
	}

	const float sp = s_state.requested_setpoint_c;
	const float kp0 = s_pid[0].kp;
	const float ki0 = s_pid[0].ki;
	const float kd0 = s_pid[0].kd;
	const float kp1 = s_pid[1].kp;
	const float ki1 = s_pid[1].ki;
	const float kd1 = s_pid[1].kd;
	const float mode4_base_ms = s_state.mode4_heat_time_base_ms;

	xSemaphoreGive(s_state_lock);

	if (!command_accepted) {
		if (cmd->type == COMM_COMMAND_HTIME) {
			ESP_LOGW(TAG,
					 "reject HTIME=%.3fms, valid range is %.1f..%.1fms",
					 cmd->value,
					 APP_MODE4_HEAT_TIME_MIN_MS,
					 APP_MODE4_HEAT_TIME_MAX_MS);
		} else if (cmd->type == COMM_COMMAND_SETPOINT || cmd->type == COMM_COMMAND_KP ||
				   cmd->type == COMM_COMMAND_KI || cmd->type == COMM_COMMAND_KD ||
				   cmd->type == COMM_COMMAND_ILIMIT) {
			ESP_LOGW(TAG, "reject non-finite command: cmd=%d value=%f", cmd->type, cmd->value);
		}
		return;
	}
	if (!print_status_log) {
		return;
	}

	// 只有实际改变配置的命令才打印完整状态，避免心跳日志占用串口和任务时间。
	ESP_LOGI(TAG,
			 "cmd=%d value=%.3f sp=%.2f mode4_base=%.1fms kp0=%.2f ki0=%.2f kd0=%.2f kp1=%.2f ki1=%.2f kd1=%.2f",
			 cmd->type,
			 cmd->value,
			 sp,
			 mode4_base_ms,
			 kp0,
			 ki0,
			 kd0,
			 kp1,
			 ki1,
			 kd1);
}

#if FEATURE_HEATING_MODE == 3
static void update_mode3_setpoints(float proc_temp_ch0, bool valid_ch0,
                                    float proc_temp_ch1, bool valid_ch1,
                                    float requested_sp,
                                    float *out_sp_ch0, float *out_sp_ch1) {
    if (out_sp_ch0 == NULL || out_sp_ch1 == NULL) {
        return;
    }

    switch (s_mode3_state) {
        case MODE3_CH0_HEAT:
            *out_sp_ch0 = APP_CYCLIC_SETPOINT2_C;
            *out_sp_ch1 = requested_sp;
            if (valid_ch0 && fabsf(proc_temp_ch0 - APP_CYCLIC_SETPOINT2_C) <= APP_CYCLIC_HOLD_THRESHOLD_C) {
                s_mode3_state = MODE3_CH0_COOL;
            }
            break;

        case MODE3_CH0_COOL:
            *out_sp_ch0 = requested_sp;
            *out_sp_ch1 = requested_sp;
            if (valid_ch0 && proc_temp_ch0 <= APP_MODE3_TRIG_TEMP_C) {
                s_mode3_state = MODE3_CH1_HEAT;
            }
            break;

        case MODE3_CH1_HEAT:
            *out_sp_ch0 = requested_sp;
            *out_sp_ch1 = APP_CYCLIC_SETPOINT2_C;
            if (valid_ch1 && fabsf(proc_temp_ch1 - APP_CYCLIC_SETPOINT2_C) <= APP_CYCLIC_HOLD_THRESHOLD_C) {
                s_mode3_state = MODE3_CH1_COOL;
            }
            break;

        case MODE3_CH1_COOL:
            *out_sp_ch0 = requested_sp;
            *out_sp_ch1 = requested_sp;
            if (valid_ch1 && proc_temp_ch1 <= APP_MODE3_TRIG_TEMP_C) {
                s_mode3_state = MODE3_CH0_HEAT;
            }
            break;

        default:
            s_mode3_state = MODE3_CH0_HEAT;
            break;
    }
}
#endif

#if FEATURE_HEATING_MODE == 4
static float mode4_clampf(float value, float min_value, float max_value) {
	// 模式 4 内部使用的通用限幅，保证时长和单次修正量始终处于安全范围。
	if (value < min_value) {
		return min_value;
	}
	if (value > max_value) {
		return max_value;
	}
	return value;
}

static void mode4_reset_group(uint32_t group,
							  float low_temp_c,
							  float heat_time_base_ms,
							  uint32_t config_revision) {
	// SP/HTIME 变更后必须同时复位状态机、PID 历史项和自适应学习结果。
	// 基准值和工作值在此刻相同，之后只有自适应算法可以修改工作值。
	mode4_group_state_t *ctx = &s_mode4[group];
	ctx->state = MODE4_STABILIZE;
	ctx->state_enter_ms = app_now_ms();
	ctx->stable_start_ms = 0U;
	ctx->stable_timer_active = false;
	ctx->applied_config_revision = config_revision;
	ctx->configured_low_c = low_temp_c;
	ctx->heat_time_base_ms = heat_time_base_ms;
	ctx->heat_time_ms = heat_time_base_ms;
	ctx->heat_start_temp_c = NAN;
	ctx->heat_end_temp_c = NAN;
	ctx->peak_temp_c = NAN;
	ctx->converged = false;
	ctx->invalid_range_warned = false;
	ctx->saturation_warned = false;
	ctrl_pid_reset(&s_pid[group]);

	ESP_LOGI(TAG,
			 "mode4 group%lu reset: T_low=%.2fC T_high=%.2fC base=%.1fms revision=%lu",
			 (unsigned long)group,
			 low_temp_c,
			 APP_CYCLIC_SETPOINT2_C,
			 heat_time_base_ms,
			 (unsigned long)config_revision);
}

static void mode4_return_to_stabilize(uint32_t group) {
	// 保护条件或配置异常中断当前脉冲时，统一回到低温稳定阶段。
	// 只在从 HEAT/COOL 真正跨状态返回时清空 PID；若已在 STABILIZE，保留积分项以便 PID 继续正常工作。
	// 稳定计时的活动标志始终清除：任何传感器失效、欠压或 OTA 扰动后都必须重新完整确认稳定保持期。
	// 若欠压在阈值附近持续抖动，该计时可能反复清零而无期限地阻止全功率脉冲，这是有意的安全收紧。
	mode4_group_state_t *ctx = &s_mode4[group];
	if (ctx->state != MODE4_STABILIZE) {
		ctx->state = MODE4_STABILIZE;
		ctrl_pid_reset(&s_pid[group]);
	}
	ctx->stable_timer_active = false;
}

static void mode4_enter_cool(uint32_t group, uint32_t now_ms, float process_temp_c, const char *reason) {
	// 记录脉冲末端温度供升温斜率计算，并从当前温度开始追踪断电后的惯性峰值。
	mode4_group_state_t *ctx = &s_mode4[group];
	ctx->state = MODE4_COOL;
	ctx->state_enter_ms = now_ms;
	ctx->heat_end_temp_c = process_temp_c;
	if (!isfinite(ctx->peak_temp_c) || process_temp_c > ctx->peak_temp_c) {
		ctx->peak_temp_c = process_temp_c;
	}

	ESP_LOGI(TAG,
			 "mode4 group%lu HEAT->COOL (%s): start=%.2fC end=%.2fC peak=%.2fC pulse=%.1fms",
			 (unsigned long)group,
			 reason,
			 ctx->heat_start_temp_c,
			 ctx->heat_end_temp_c,
			 ctx->peak_temp_c,
			 ctx->heat_time_ms);
}

static void mode4_adapt_heat_time(uint32_t group) {
	mode4_group_state_t *ctx = &s_mode4[group];
	const float peak_error_c = APP_CYCLIC_SETPOINT2_C - ctx->peak_temp_c;

	// 峰值进入容差带后将本组标记为已收敛，直到 SP/HTIME 命令复位前不再改变脉冲宽度。
	if (isfinite(ctx->peak_temp_c) && fabsf(peak_error_c) <= APP_MODE4_PEAK_TOL_C) {
		ctx->converged = true;
		ESP_LOGI(TAG,
				 "mode4 group%lu converged: peak=%.2fC target=%.2fC heat_time=%.1fms",
				 (unsigned long)group,
				 ctx->peak_temp_c,
				 APP_CYCLIC_SETPOINT2_C,
				 ctx->heat_time_ms);
		return;
	}

#if APP_MODE4_ADAPT_ENABLE
	if (ctx->converged || !isfinite(ctx->heat_start_temp_c) ||
		!isfinite(ctx->heat_end_temp_c) || !isfinite(ctx->peak_temp_c)) {
		return;
	}

	// 严格按需求用“脉冲末温度-脉冲起始温度”除以当前工作时长得到实测斜率。
	// 斜率为零或负值意味着测量或加热链路异常，此时不可用牛顿步长进行除法。
	const float slope_c_per_ms =
		(ctx->heat_end_temp_c - ctx->heat_start_temp_c) / ctx->heat_time_ms;
	if (!isfinite(slope_c_per_ms) || slope_c_per_ms <= 0.000001f) {
		ESP_LOGW(TAG,
				 "mode4 group%lu cannot adapt: invalid heating slope %.6fC/ms",
				 (unsigned long)group,
				 slope_c_per_ms);
		return;
	}

	const float quantum_ms = (float)APP_CONTROL_PERIOD_MS;
	float delta_ms = peak_error_c / slope_c_per_ms;
	// 脉冲只能按控制周期落地，因此先四舍五入到 20ms 的整数倍。
	delta_ms = roundf(delta_ms / quantum_ms) * quantum_ms;
	if (fabsf(delta_ms) < quantum_ms) {
		return;
	}

	// 将 30% 步长上限向下量化到完整控制周期，避免限幅后反而产生非 20ms 整数倍。
	const float max_step_ms =
		floorf((ctx->heat_time_ms * APP_MODE4_ADAPT_MAX_STEP_RATIO) / quantum_ms) * quantum_ms;
	if (max_step_ms < quantum_ms) {
		return;
	}
	delta_ms = mode4_clampf(delta_ms, -max_step_ms, max_step_ms);

	const float old_heat_time_ms = ctx->heat_time_ms;
	ctx->heat_time_ms = mode4_clampf(
		ctx->heat_time_ms + delta_ms,
		APP_MODE4_HEAT_TIME_MIN_MS,
		APP_MODE4_HEAT_TIME_MAX_MS);

	ESP_LOGI(TAG,
			 "mode4 group%lu adapt: peak=%.2fC slope=%.5fC/ms delta=%.1fms heat_time %.1f->%.1fms",
			 (unsigned long)group,
			 ctx->peak_temp_c,
			 slope_c_per_ms,
			 delta_ms,
			 old_heat_time_ms,
			 ctx->heat_time_ms);

	if (ctx->heat_time_ms >= APP_MODE4_HEAT_TIME_MAX_MS &&
		ctx->peak_temp_c < (APP_CYCLIC_SETPOINT2_C - APP_MODE4_PEAK_TOL_C) &&
		!ctx->saturation_warned) {
		ctx->saturation_warned = true;
		ESP_LOGW(TAG,
				 "mode4 group%lu heat time saturated at %.1fms but peak %.2fC is below target %.2fC",
				 (unsigned long)group,
				 ctx->heat_time_ms,
				 ctx->peak_temp_c,
				 APP_CYCLIC_SETPOINT2_C);
	}
#endif
}

static float mode4_update_group(uint32_t group,
							float process_temp_c,
							bool process_valid,
							bool output_inhibited,
							float low_temp_c,
							float heat_time_base_ms,
							uint32_t config_revision,
							float dt_s) {
	mode4_group_state_t *ctx = &s_mode4[group];
	const uint32_t now_ms = app_now_ms();

	// 配置命令是强制复位事件；此外，心跳保护若改变了有效低温点，也必须从稳定阶段重新开始。
	if (ctx->applied_config_revision != config_revision ||
		ctx->configured_low_c != low_temp_c ||
		ctx->heat_time_base_ms != heat_time_base_ms) {
		mode4_reset_group(group, low_temp_c, heat_time_base_ms, config_revision);
	}

	// 欠压或 OTA 不仅要把本周期输出压为零，还要取消正在进行的脉冲计时。
	// 否则保护解除后可能继续半个旧脉冲，而本轮升温斜率也会被停电时间污染。
	if (output_inhibited) {
		mode4_return_to_stabilize(group);
		return 0.0f;
	}

	// 任一温度传感器组失效时立即返回 0ms，并丢弃当前脉冲进度。
	// 恢复测温后先重新 PID 稳定，不会继续一个时间基准已经失真的剩余脉冲。
	if (!process_valid || !isfinite(process_temp_c)) {
		mode4_return_to_stabilize(group);
		return 0.0f;
	}

	// T_high 必须严格大于 T_low。不合法时只保留低温 PID，禁止进入全功率脉冲。
	const bool temperature_range_valid =
		isfinite(low_temp_c) && isfinite(APP_CYCLIC_SETPOINT2_C) &&
		APP_CYCLIC_SETPOINT2_C > low_temp_c;
	if (!temperature_range_valid) {
		mode4_return_to_stabilize(group);
		if (!ctx->invalid_range_warned) {
			ctx->invalid_range_warned = true;
			ESP_LOGW(TAG,
					 "mode4 group%lu disabled: T_high %.2fC must be greater than T_low %.2fC; PID-only fallback",
					 (unsigned long)group,
					 APP_CYCLIC_SETPOINT2_C,
					 low_temp_c);
		}
		if (!isfinite(low_temp_c)) {
			return 0.0f;
		}
		ctrl_pid_set_setpoint(&s_pid[group], low_temp_c);
		return ctrl_pid_update(&s_pid[group], process_temp_c, dt_s);
	}
	ctx->invalid_range_warned = false;

	switch (ctx->state) {
		case MODE4_STABILIZE: {
			ctrl_pid_set_setpoint(&s_pid[group], low_temp_c);
			const float pid_output_ms = ctrl_pid_update(&s_pid[group], process_temp_c, dt_s);
			if (fabsf(process_temp_c - low_temp_c) <= APP_CYCLIC_HOLD_THRESHOLD_C) {
				if (!ctx->stable_timer_active) {
					ctx->stable_start_ms = now_ms;
					ctx->stable_timer_active = true;
				} else if ((now_ms - ctx->stable_start_ms) >= APP_CYCLIC_HOLD_TIME_MS) {
					// 进入脉冲前清空 PID 积分和历史误差，后续 HEAT/COOL 阶段完全不调用 PID。
					ctrl_pid_reset(&s_pid[group]);
					ctx->state = MODE4_HEAT;
					ctx->state_enter_ms = now_ms;
					ctx->stable_timer_active = false;
					ctx->heat_start_temp_c = process_temp_c;
					ctx->heat_end_temp_c = process_temp_c;
					ctx->peak_temp_c = process_temp_c;
					ESP_LOGI(TAG,
							 "mode4 group%lu STABILIZE->HEAT: temp=%.2fC pulse=%.1fms",
							 (unsigned long)group,
							 process_temp_c,
							 ctx->heat_time_ms);
					return APP_PWM_PERIOD_MS;
				}
			} else {
				// 稳定计时必须连续，一旦离开容差带立即清零。
				ctx->stable_timer_active = false;
			}
			return pid_output_ms;
		}

		case MODE4_HEAT: {
			if (process_temp_c > ctx->peak_temp_c) {
				ctx->peak_temp_c = process_temp_c;
			}
			const uint32_t elapsed_ms = now_ms - ctx->state_enter_ms;
			if (process_temp_c >= (APP_CYCLIC_SETPOINT2_C + APP_MODE4_OVERTEMP_TRIP_C)) {
				mode4_enter_cool(group, now_ms, process_temp_c, "over-temperature trip");
				return 0.0f;
			}

			// 先检查相对超时：若任务调度异常使一次循环跨过两个边界，日志中仍能明确标记为安全超时。
			const float timeout_ms = ctx->heat_time_ms * APP_MODE4_HEAT_TIMEOUT_FACTOR;
			if ((float)elapsed_ms >= timeout_ms) {
				ESP_LOGW(TAG,
						 "mode4 group%lu heat timeout: elapsed=%lums limit=%.1fms",
						 (unsigned long)group,
						 (unsigned long)elapsed_ms,
						 timeout_ms);
				mode4_enter_cool(group, now_ms, process_temp_c, "relative timeout");
				return 0.0f;
			}
			if ((float)elapsed_ms >= ctx->heat_time_ms) {
				mode4_enter_cool(group, now_ms, process_temp_c, "pulse complete");
				return 0.0f;
			}
			return APP_PWM_PERIOD_MS;
		}

		case MODE4_COOL: {
			const uint32_t cool_elapsed_ms = now_ms - ctx->state_enter_ms;
			// 只在断电后前 500ms 更新峰值，超过窗口后冻结结果，避免冷却期噪声改写本轮峰值。
			if (cool_elapsed_ms <= APP_MODE4_PEAK_TRACK_MS && process_temp_c > ctx->peak_temp_c) {
				ctx->peak_temp_c = process_temp_c;
			}

			// 至少完整观测 500ms 热惯性窗口后才允许开始下一轮。
			// 这样即使小热容负载在 500ms 内已降到低温阈值，峰值也不会被提前截断。
			if (cool_elapsed_ms >= APP_MODE4_PEAK_TRACK_MS &&
				process_temp_c <= (low_temp_c - APP_MODE4_LOW_HYST_C)) {
				mode4_adapt_heat_time(group);
				ctx->state = MODE4_HEAT;
				ctx->state_enter_ms = now_ms;
				ctx->heat_start_temp_c = process_temp_c;
				ctx->heat_end_temp_c = process_temp_c;
				ctx->peak_temp_c = process_temp_c;
				ESP_LOGI(TAG,
						 "mode4 group%lu COOL->HEAT: temp=%.2fC next_pulse=%.1fms converged=%d",
						 (unsigned long)group,
						 process_temp_c,
						 ctx->heat_time_ms,
						 ctx->converged);
				return APP_PWM_PERIOD_MS;
			}
			return 0.0f;
		}

		default:
			mode4_reset_group(group, low_temp_c, heat_time_base_ms, config_revision);
			return 0.0f;
	}
}
#endif

static void control_task(void *arg) {
	(void)arg;
	// 离散 PID 的采样周期（秒）。
	const float dt_s = APP_CONTROL_PERIOD_MS / 1000.0f;
	const TickType_t control_period_ticks =
		(pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS) > 0) ? pdMS_TO_TICKS(APP_CONTROL_PERIOD_MS) : 1;
	// 等价于 osKernelGetTickCount + osDelayUntil，确保控制循环严格对齐固定周期。
	TickType_t sys_tick_count_ctrl = xTaskGetTickCount();

	// 任务启动后再加载运行 PID 参数：上电阶段保持 0 输出，进入控制任务后才启用调参值。
	xSemaphoreTake(s_state_lock, portMAX_DELAY);
	for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
		ctrl_pid_set_gains(&s_pid[group], APP_PID_TASK_START_KP, APP_PID_TASK_START_KI, APP_PID_TASK_START_KD);
		ctrl_pid_reset(&s_pid[group]);
	}
	xSemaphoreGive(s_state_lock);
	ESP_LOGI(TAG,
			 "pid task start gains: kp=%.2f ki=%.2f kd=%.2f",
			 APP_PID_TASK_START_KP,
			 APP_PID_TASK_START_KI,
			 APP_PID_TASK_START_KD);

	while (1) {
		app_runtime_t sample = {0};

		// 1) 在极短临界区复制最新传感器快照与控制输入。
		float requested_sp = APP_DEFAULT_SETPOINT_C;
#if FEATURE_HEATING_MODE == 4
		float mode4_heat_time_base_ms = APP_MODE4_HEAT_TIME_DEFAULT_MS;
		uint32_t mode4_config_revision = 0U;
#endif
#if FEATURE_WIRELESS_ENABLE && FEATURE_HEARTBEAT_FAILSAFE_ENABLE
		uint32_t last_hb = 0;
#endif
		bool ota_pending = false;
		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		for (uint32_t ch = 0; ch < 4; ++ch) {
			sample.ntc_temp_c[ch] = s_state.ntc_temp_c[ch];
			sample.ntc_voltage_v[ch] = s_state.ntc_voltage_v[ch];
			sample.ntc_valid[ch] = s_state.ntc_valid[ch];
		}
		sample.wf_temp_c = s_state.wf_temp_c;
		sample.wf_pressure_kpa = s_state.wf_pressure_kpa;
		sample.wf_valid = s_state.wf_valid;
		sample.supply_voltage_v = s_state.supply_voltage_v;
		sample.undervoltage = s_state.undervoltage;
		ota_pending = s_state.ota_pending;

#if FEATURE_WIRELESS_ENABLE && FEATURE_HEARTBEAT_FAILSAFE_ENABLE
		last_hb = s_state.last_heartbeat_ms;
#endif
		requested_sp = s_state.requested_setpoint_c;
#if FEATURE_HEATING_MODE == 4
		mode4_heat_time_base_ms = s_state.mode4_heat_time_base_ms;
		mode4_config_revision = s_state.mode4_config_revision;
#endif
		xSemaphoreGive(s_state_lock);

		// 2) 选择控制温度源并计算当前过程温度。
		float process_temp[APP_CONTROL_GROUPS] = {NAN, NAN};
		bool process_valid[APP_CONTROL_GROUPS] = {false, false};
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			process_valid[group] = get_group_average_temperature(
				&sample,
				group,
				&process_temp[group]);
		}

		// 3) 计算每路目标设定值与失联保护后的有效设定值。
		float target_sp[APP_CONTROL_GROUPS] = {requested_sp, requested_sp};
		float effective_sp[APP_CONTROL_GROUPS] = {requested_sp, requested_sp};
#if FEATURE_HEATING_MODE == 2
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			target_sp[group] = update_cyclic_setpoint_group(
				group,
				requested_sp,
				process_temp[group],
				process_valid[group]);
		}
#elif FEATURE_HEATING_MODE == 3
		update_mode3_setpoints(
			process_temp[0], process_valid[0],
			process_temp[1], process_valid[1],
			requested_sp,
			&target_sp[0], &target_sp[1]);
#endif
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			if (target_sp[group] != s_last_target_sp[group]) {
				ctrl_pid_reset(&s_pid[group]);
				s_last_target_sp[group] = target_sp[group];
			}
		}

#if FEATURE_WIRELESS_ENABLE
		#if FEATURE_HEARTBEAT_FAILSAFE_ENABLE
			const uint32_t now_ms = app_now_ms();
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				effective_sp[group] = ctrl_failsafe_effective_setpoint(
					&s_failsafe,
					now_ms,
					last_hb,
					target_sp[group]);
			}
		#else
			// 关闭心跳失联保护时，始终采用请求设定值。
			s_failsafe.safe_mode = false;
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				effective_sp[group] = target_sp[group];
			}
		#endif
#else
		// 无线上位机关闭时不依赖心跳，避免单机调试被误判为失联保护。
		s_failsafe.safe_mode = false;
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			effective_sp[group] = target_sp[group];
		}
#endif

		// 4) 模式 1~3 使用通用 PID 输出；模式 4 由三态状态机决定 PID/100%/0% 输出。
		// 两条路径最后都经过欠压和 OTA 硬关断，传感器无效时也不会向 PWM 驱动下发非零值。
		float pwm_on_ms[APP_CONTROL_GROUPS] = {0.0f, 0.0f};
		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
#if FEATURE_HEATING_MODE == 4
			pwm_on_ms[group] = mode4_update_group(
				group,
				process_temp[group],
				process_valid[group],
				sample.undervoltage || ota_pending,
				effective_sp[group],
				mode4_heat_time_base_ms,
				mode4_config_revision,
				dt_s);
#else
			if (process_valid[group]) {
				ctrl_pid_set_setpoint(&s_pid[group], effective_sp[group]);
				pwm_on_ms[group] = ctrl_pid_update(&s_pid[group], process_temp[group], dt_s);
			}
#endif

			// 欠压或 OTA 挂起期间无条件覆盖为 0，模式 4 的全功率脉冲也不例外。
			if (sample.undervoltage || ota_pending) {
				pwm_on_ms[group] = 0.0f;
			}

			s_state.process_temp_c[group] = process_temp[group];
			s_state.effective_setpoint_c[group] = effective_sp[group];
			s_state.pwm_on_ms[group] = pwm_on_ms[group];
		}
		xSemaphoreGive(s_state_lock);

		// 5) 将控制输出写入 PWM 驱动。
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			const uint8_t pwm_channel = s_group_map[group].pwm_channel;
			if (process_valid[group]) {
				periph_pwm_set_on_time_ms_ch(pwm_channel, pwm_on_ms[group]);
			} else {
				periph_pwm_force_off_ch(pwm_channel);
			}
		}
		vTaskDelayUntil(&sys_tick_count_ctrl, control_period_ticks);
	}
}

static void telemetry_send(uint8_t cmd, const uint8_t *payload, size_t payload_len) {
	// 统一封帧发送：组帧失败时直接丢弃当前数据。
	uint8_t frame[96] = {0};
	const size_t frame_len =
		comm_protocol_build_frame(cmd, payload, (uint8_t)payload_len, frame, sizeof(frame));
	if (frame_len == 0) {
		return;
	}

#if FEATURE_UPLOAD_ENABLE && FEATURE_WIRELESS_ENABLE
	// 上报发送失败不阻塞主流程，由下次周期继续发送。
	(void)comm_udp_send(frame, frame_len);
#endif
}

static void telemetry_task(void *arg) {
	(void)arg;

	while (1) {
		// 先复制一份快照，减少锁持有时间。
		app_runtime_t snapshot = {0};
		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		snapshot = s_state;
		xSemaphoreGive(s_state_lock);

		float pressure_kpa_1 = NAN;
		float pressure_kpa_2 = NAN;
		const char *pressure_src = "OFF";

#if APP_PRESSURE_SOURCE_WF
		if (snapshot.wf_valid) {
			pressure_kpa_1 = snapshot.wf_pressure_kpa;
		}
		pressure_src = "WF";
#elif APP_PRESSURE_SOURCE_DC
		if ((snapshot.pressure_mask & 0x01) != 0) {
			pressure_kpa_1 = snapshot.dc_pressure_kpa_ch1;
		}
		if ((snapshot.pressure_mask & 0x02) != 0) {
			pressure_kpa_2 = snapshot.dc_pressure_kpa_ch2;
		}
		pressure_src = "DC";
#endif

		// USB 日志实时输出。
		ESP_LOGI(TAG,
				 "T0=%.2f V0=%.3f T1=%.2f V1=%.3f T2=%.2f V2=%.3f T3=%.2f V3=%.3f WF_T=%.2f P1=%.2f P2=%.2f Psrc=%s V=%.2f PWM0=%.1f PWM1=%.1f SP0=%.2f SP1=%.2f SAFE=%d",
				 snapshot.ntc_temp_c[0],
				 snapshot.ntc_voltage_v[0],
				 snapshot.ntc_temp_c[1],
				 snapshot.ntc_voltage_v[1],
				 snapshot.ntc_temp_c[2],
				 snapshot.ntc_voltage_v[2],
				 snapshot.ntc_temp_c[3],
				 snapshot.ntc_voltage_v[3],
				 snapshot.wf_temp_c,
				 pressure_kpa_1,
				 pressure_kpa_2,
				 pressure_src,
				 snapshot.supply_voltage_v,
				 snapshot.pwm_on_ms[0],
				 snapshot.pwm_on_ms[1],
				 snapshot.effective_setpoint_c[0],
				 snapshot.effective_setpoint_c[1],
				 s_failsafe.safe_mode);

#if FEATURE_NTC_CH0_ENABLE || FEATURE_NTC_CH1_ENABLE || FEATURE_NTC_CH2_ENABLE || FEATURE_NTC_CH3_ENABLE
		// 先按控制组计算平均温度，再根据映射表存入对应 PWM 下标。
		// group0 当前映射到 PWM1、group1 映射到 PWM0，显式重排可保持通信协议原有的 PWM0/PWM1 顺序。
		float pwm_temp_c[APP_CONTROL_GROUPS] = {0.0f, 0.0f};
		bool pwm_temp_valid[APP_CONTROL_GROUPS] = {false, false};
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			float group_temp_c = NAN;
			const bool group_valid = get_group_average_temperature(&snapshot, group, &group_temp_c);
			const uint8_t pwm_channel = s_group_map[group].pwm_channel;
			pwm_temp_c[pwm_channel] = group_valid ? group_temp_c : 0.0f;
			pwm_temp_valid[pwm_channel] = group_valid;
		}

		uint8_t payload_ntc[8] = {0};
		const size_t ntc_len = comm_protocol_pack_ntc_payload(
			pwm_temp_valid[0],
			pwm_temp_c[0],
			pwm_temp_valid[1],
			pwm_temp_c[1],
			payload_ntc,
			sizeof(payload_ntc));
		if (ntc_len > 0) {
			telemetry_send(COMM_CMD_NTC, payload_ntc, ntc_len);
		}
#endif

#if APP_PRESSURE_SOURCE_WF
		// WF5803F 温压帧。
		if (snapshot.wf_valid) {
			uint8_t payload_wf[8] = {0};
			const size_t wf_len = comm_protocol_pack_wf_payload(
				snapshot.wf_temp_c, snapshot.wf_pressure_kpa, payload_wf, sizeof(payload_wf));
			if (wf_len > 0) {
				telemetry_send(COMM_CMD_WF5803F, payload_wf, wf_len);
			}
		}
#endif

#if APP_PRESSURE_SOURCE_DC
		// DC 压力帧。
		if (snapshot.pressure_mask != 0) {
			uint8_t payload_p[16] = {0}; // 确保缓冲区足够大
			const size_t p_len = comm_protocol_pack_dynamic_pressure_payload(
				payload_p, 
				snapshot.pressure_mask, 
				snapshot.dc_pressure_kpa_ch1, 
				snapshot.dc_pressure_kpa_ch2
			);

			if (p_len > 1) {
				telemetry_send(COMM_CMD_PRESSURE, payload_p, p_len);
			}
		}
#endif

#if FEATURE_VOLTAGE_MONITOR_ENABLE
		// 电压状态帧。
		uint8_t payload_v[4] = {0};
		const size_t v_len = comm_protocol_pack_voltage_payload(
			snapshot.supply_voltage_v, snapshot.undervoltage, payload_v, sizeof(payload_v));
		if (v_len > 0) {
			telemetry_send(COMM_CMD_VOLTAGE, payload_v, v_len);
		}
#endif

#if FEATURE_PID_OUT_ENABLE
		// s_state.pwm_on_ms[] 以“控制组”为下标，而通信协议要求物理 PWM0 在前、PWM1 在后。
		// 通过统一映射表显式转换，即使以后调整控制组顺序，遥测帧也不会把两路发反。
		float pwm_output_ms[2] = {0.0f, 0.0f};
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			const uint8_t pwm_channel = s_group_map[group].pwm_channel;
			pwm_output_ms[pwm_channel] = snapshot.pwm_on_ms[group];
		}

		uint8_t payload_pid[8] = {0};
		const size_t pid_len = comm_protocol_pack_pid_out_payload_2ch(
			pwm_output_ms[0],
			pwm_output_ms[1],
			payload_pid,
			sizeof(payload_pid));
		if (pid_len == sizeof(payload_pid)) {
			telemetry_send(COMM_CMD_PID_OUT, payload_pid, pid_len);
		}
#endif

		// 控制上报频率。
		vTaskDelay(pdMS_TO_TICKS(APP_TELEMETRY_PERIOD_MS));
	}
}

#if FEATURE_WIRELESS_ENABLE
static void udp_command_task(void *arg) {
	(void)arg;
	// 复用固定缓冲接收上位机文本命令。
	char line[96] = {0};

	while (1) {
		// 轮询接收 UDP 命令，超时会返回 0。
		const int len = comm_udp_receive_line(line, sizeof(line));
		if (len > 0) {
			comm_command_t cmd = {0};
			// 解析成功后写入系统状态。
			if (comm_command_parse_line(line, &cmd)) {
				apply_command(&cmd);
			}
		}
	}
}
#endif

static void console_command_task(void *arg) {
	(void)arg;
	// 通过 USB 串口输入调试命令。
	char line[96] = {0};

	while (1) {
		// 非阻塞读取失败时短暂休眠，避免空转占用。
		if (fgets(line, sizeof(line), stdin) == NULL) {
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}

		comm_command_t cmd = {0};
		// 复用同一套命令解析逻辑。
		if (comm_command_parse_line(line, &cmd)) {
			apply_command(&cmd);
		}
	}
}

static void ota_task(void *arg) {
	(void)arg;

	while (1) {
		// 读取并消费一次 OTA 请求标志。
		bool start_ota = false;
		float temp_samples[APP_CONTROL_GROUPS] = {NAN, NAN};

		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		if (s_state.ota_pending) {
			start_ota = true;
			temp_samples[0] = s_state.process_temp_c[0];
			temp_samples[1] = s_state.process_temp_c[1];
			s_state.ota_pending = false;
		}
		xSemaphoreGive(s_state_lock);

		if (start_ota) {
			float current_temp_c = NAN;
			if (isfinite(temp_samples[0])) {
				current_temp_c = temp_samples[0];
			}
			if (isfinite(temp_samples[1])) {
				if (!isfinite(current_temp_c) || temp_samples[1] > current_temp_c) {
					current_temp_c = temp_samples[1];
				}
			}
			// 在安全温度下执行 OTA，失败仅记录日志。
			const esp_err_t err = sys_ota_perform_if_safe(APP_OTA_URL, current_temp_c);
			if (err != ESP_OK) {
				ESP_LOGW(TAG, "ota request failed: %s", esp_err_to_name(err));
			}
		}

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void app_main(void) {
	// 1) 初始化基础系统。
	ESP_ERROR_CHECK(init_nvs());
	sys_ota_mark_app_valid();

	// 2) 创建全局状态锁。
	s_state_lock = xSemaphoreCreateMutex();
	if (s_state_lock == NULL) {
		ESP_LOGE(TAG, "failed to create state mutex");
		return;
	}

	// 初始化 NTC 滑动窗口状态。
	ntc_filter_reset(&s_ntc_filter);

	// 3) 初始化运行态与控制参数。
	runtime_init();
	// 上电固定为 0/0/0，确保烧录与启动阶段不输出控制量。
	for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
		ctrl_pid_init(
			&s_pid[group],
			0.0f,
			0.0f,
			0.0f,
			APP_DEFAULT_SETPOINT_C);
		ctrl_pid_set_integral_limit(&s_pid[group], APP_PID_ILIMIT_DEFAULT);
	}
	ctrl_failsafe_init(&s_failsafe, APP_HEARTBEAT_TIMEOUT_MS, APP_SAFE_SETPOINT_C);

	// 4) 初始化外设。
	ESP_ERROR_CHECK(periph_i2c_init());
	ESP_ERROR_CHECK(periph_pwm_init());

	// 5) 启动通讯（若失败不阻断主控任务）。
#if FEATURE_WIRELESS_ENABLE
	const esp_err_t comm_err = comm_udp_start();
	if (comm_err != ESP_OK) {
		ESP_LOGW(TAG, "udp not ready: %s", esp_err_to_name(comm_err));
	}
#else
	ESP_LOGI(TAG, "wireless disabled by FEATURE_WIRELESS_ENABLE");
#endif

	// 6) 启动各业务任务。
	// 双核分工：通信相关任务（UDP、控制台）放在 core_comm，控制任务放在 core_ctrl，OTA 任务放在 core_comm 避免干扰控制。
	const BaseType_t core_comm = 0;
	const BaseType_t core_ctrl = 1;

	// NTC 后台采样任务：持续更新滑动窗口，降低控制链路等待。
	xTaskCreatePinnedToCore(sampling_task, "sampling_task", 4096, NULL, 7, NULL, core_ctrl);
	// 控制任务优先级最高，且独占一个核心，确保控制响应的实时性和稳定性。
	xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, 8, NULL, core_ctrl);
	// 遥测任务优先级适中，保证稳定输出同时不干扰控制。
	// 数据发送任务，（如 UDP）优先级同遥测，避免发送阻塞导致数据积压。
	xTaskCreatePinnedToCore(telemetry_task, "telemetry_task", 4096, NULL, 5, NULL, core_comm);
#if FEATURE_WIRELESS_ENABLE
	// UDP 命令接收任务（负责接收和处理 UDP 命令），归入 send 核心。
	xTaskCreatePinnedToCore(udp_command_task, "udp_cmd_task", 4096, NULL, 6, NULL, core_comm);
#endif
	// 控制台命令任务（负责处理串口输入的调试命令），优先级同 UDP 命令，归入 control 核心。
	xTaskCreatePinnedToCore(console_command_task, "console_cmd_task", 4096, NULL, 6, NULL, core_comm);
	// OTA 任务，归入 other 核心。
	xTaskCreatePinnedToCore(ota_task, "ota_task", 6144, NULL, 4, NULL, core_comm);
}
