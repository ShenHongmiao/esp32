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
static uint8_t s_cyclic_stage[APP_CONTROL_GROUPS] = {1, 1};
static uint32_t s_cyclic_hold_start_ms[APP_CONTROL_GROUPS];
static float s_last_target_sp[APP_CONTROL_GROUPS] = {APP_DEFAULT_SETPOINT_C, APP_DEFAULT_SETPOINT_C};

// 模式 3 状态机定义：双通道互锁交替。
typedef enum {
    MODE3_CH0_HEAT,   // CH0 加热（目标高温），CH1 冷却（目标低温）
    MODE3_CH0_COOL,   // CH0 已到高温，等待 CH0 降温到触发点
    MODE3_CH1_HEAT,   // CH1 加热（目标高温），CH0 冷却（目标低温）
    MODE3_CH1_COOL,   // CH1 已到高温，等待 CH1 降温到触发点
} mode3_state_t;
static mode3_state_t s_mode3_state = MODE3_CH0_HEAT;

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
	s_state.dc_pressure_kpa_ch1 = 0.0f;
	s_state.dc_pressure_kpa_ch2 = 0.0f;
	s_state.pressure_mask = 0;
}

static float select_process_temperature_group(const app_runtime_t *sample, uint8_t primary, uint8_t secondary) {
	// 控制温度源优先级：主通道优先，备用通道回退。
	if (sample->ntc_valid[primary]) {
		return sample->ntc_temp_c[primary];
	}
	if (sample->ntc_valid[secondary]) {
		return sample->ntc_temp_c[secondary];
	}
	return NAN;
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
		if (s_cyclic_hold_start_ms[group] == 0U) {
			s_cyclic_hold_start_ms[group] = now_ms;
		} else if ((now_ms - s_cyclic_hold_start_ms[group]) >= APP_CYCLIC_HOLD_TIME_MS) {
			s_cyclic_stage[group] = (s_cyclic_stage[group] == 1U) ? 2U : 1U;
			s_cyclic_hold_start_ms[group] = 0U;
			target = (s_cyclic_stage[group] == 2U) ? APP_CYCLIC_SETPOINT2_C : stage1_setpoint;
		}
	} else {
		s_cyclic_hold_start_ms[group] = 0U;
	}

	return target;
}

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
	// 读取 DC 电压型压力传感器。
	float local_kpa_ch1 = 0.0f;
	float local_kpa_ch2 = 0.0f;
	uint8_t local_mask = 0;

#if FEATURE_PRESSURE_DC_CH1
	uint16_t raw1 = 0;
	if (periph_adc_read_raw12(APP_EXT_ADC_CMD_Press1, &raw1) == ESP_OK) {
		float v1 = periph_adc_raw12_to_voltage(raw1, APP_ADC_VREF_V) * APP_PRESSURE_DC_VOUT_SCALE;
		float p1 = periph_pressure_dc_voltage_to_kpa(v1);
		if (p1 < 0.0f) {
			p1 = 0.0f;
		}
		local_kpa_ch1 = p1;
		local_mask |= 0x01;
	}
#endif

#if FEATURE_PRESSURE_DC_CH2
	uint16_t raw2 = 0;
	if (periph_adc_read_raw12(APP_EXT_ADC_CMD_Press2, &raw2) == ESP_OK) {
		float v2 = periph_adc_raw12_to_voltage(raw2, APP_ADC_VREF_V) * APP_PRESSURE_DC_VOUT_SCALE;
		float p2 = periph_pressure_dc_voltage_to_kpa(v2);
		if (p2 < 0.0f) {
			p2 = 0.0f;
		}
		local_kpa_ch2 = p2;
		local_mask |= 0x02;
	}
#endif

	xSemaphoreTake(s_state_lock, portMAX_DELAY);
	s_state.dc_pressure_kpa_ch1 = local_kpa_ch1;
	s_state.dc_pressure_kpa_ch2 = local_kpa_ch2;
	s_state.pressure_mask = local_mask;
	xSemaphoreGive(s_state_lock);
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

#if FEATURE_NTC_CH0_ENABLE
		sampled_valid[0] =
			sample_ntc_voltage_channel(0U, APP_EXT_ADC_CMD_NTC0, &sampled_voltage_v[0]);
#endif
#if FEATURE_NTC_CH1_ENABLE
		sampled_valid[1] =
			sample_ntc_voltage_channel(1U, APP_EXT_ADC_CMD_NTC1, &sampled_voltage_v[1]);
#endif
#if FEATURE_NTC_CH2_ENABLE
		sampled_valid[2] =
			sample_ntc_voltage_channel(2U, APP_EXT_ADC_CMD_NTC2, &sampled_voltage_v[2]);
#endif
#if FEATURE_NTC_CH3_ENABLE
		sampled_valid[3] =
			sample_ntc_voltage_channel(3U, APP_EXT_ADC_CMD_NTC3, &sampled_voltage_v[3]);
#endif

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
		xSemaphoreGive(s_state_lock);

		vTaskDelayUntil(&sys_tick_count_sample, sample_period_ticks);
	}
}

static void apply_command(const comm_command_t *cmd) {
	// 命令为空时直接忽略。
	if (cmd == NULL) {
		return;
	}

	// 任何合法命令都视为“收到心跳”。
	xSemaphoreTake(s_state_lock, portMAX_DELAY);
	s_state.last_heartbeat_ms = app_now_ms();

	// 根据命令类型更新控制目标或控制器参数。
	switch (cmd->type) {
		case COMM_COMMAND_HEARTBEAT:
			break;
		case COMM_COMMAND_SETPOINT:
			s_state.requested_setpoint_c = cmd->value;
			break;
		case COMM_COMMAND_KP:
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				s_pid[group].kp = cmd->value;
			}
			break;
		case COMM_COMMAND_KI:
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				if (s_pid[group].ki != cmd->value) {
					s_pid[group].ki = cmd->value;
					s_pid[group].integral = 0.0f;
				}
			}
			break;
		case COMM_COMMAND_KD:
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				s_pid[group].kd = cmd->value;
			}
			break;
		case COMM_COMMAND_ILIMIT:
			for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
				ctrl_pid_set_integral_limit(&s_pid[group], cmd->value);
			}
			break;
		case COMM_COMMAND_OTA:
			s_state.ota_pending = true;
			break;
		default:
			break;
	}

	const float sp = s_state.requested_setpoint_c;
	const float kp0 = s_pid[0].kp;
	const float ki0 = s_pid[0].ki;
	const float kd0 = s_pid[0].kd;
	const float kp1 = s_pid[1].kp;
	const float ki1 = s_pid[1].ki;
	const float kd1 = s_pid[1].kd;

	xSemaphoreGive(s_state_lock);

	// 打印命令和当前关键参数，便于联调追踪。
	ESP_LOGI(TAG,
			 "cmd=%d value=%.3f sp=%.2f kp0=%.2f ki0=%.2f kd0=%.2f kp1=%.2f ki1=%.2f kd1=%.2f",
			 cmd->type,
			 cmd->value,
			 sp,
			 kp0,
			 ki0,
			 kd0,
			 kp1,
			 ki1,
			 kd1);
}

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
#if FEATURE_WIRELESS_ENABLE
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

#if FEATURE_WIRELESS_ENABLE
		last_hb = s_state.last_heartbeat_ms;
#endif
		requested_sp = s_state.requested_setpoint_c;
		xSemaphoreGive(s_state_lock);

		// 2) 选择控制温度源并计算当前过程温度。
		float process_temp[APP_CONTROL_GROUPS] = {NAN, NAN};
		bool process_valid[APP_CONTROL_GROUPS] = {false, false};
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			float sum = 0.0f;
			int count = 0;
			const uint8_t primary = s_group_map[group].primary_ntc;
			const uint8_t secondary = s_group_map[group].secondary_ntc;
			if (sample.ntc_valid[primary]) {
				sum += sample.ntc_temp_c[primary];
				count++;
			}
			if (sample.ntc_valid[secondary]) {
				sum += sample.ntc_temp_c[secondary];
				count++;
			}
			if (count > 0) {
				process_temp[group] = sum / (float)count;
				process_valid[group] = true;
			}
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

		// 4) 温度有效时执行 PID；无效通道仅关断对应 PWM。
		float pwm_on_ms[APP_CONTROL_GROUPS] = {0.0f, 0.0f};
		xSemaphoreTake(s_state_lock, portMAX_DELAY);
		for (uint32_t group = 0; group < APP_CONTROL_GROUPS; ++group) {
			if (process_valid[group]) {
				ctrl_pid_set_setpoint(&s_pid[group], effective_sp[group]);
				pwm_on_ms[group] = ctrl_pid_update(&s_pid[group], process_temp[group], dt_s, NULL);

				// 欠压或 OTA 挂起期间，强制输出为 0。
				if (sample.undervoltage || ota_pending) {
					pwm_on_ms[group] = 0.0f;
				}
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
		// NTC 数据帧：改为上报 PWM 通道的平均反馈温度。
		float pwm_ch1_avg = 0.0f;
		bool pwm_ch1_valid = false;
		float pwm_ch0_avg = 0.0f;
		bool pwm_ch0_valid = false;

		float sum = 0.0f;
		int count = 0;
#if FEATURE_NTC_CH0_ENABLE
		if (snapshot.ntc_valid[0]) {
			sum += snapshot.ntc_temp_c[0];
			count++;
		}
#endif
#if FEATURE_NTC_CH1_ENABLE
		if (snapshot.ntc_valid[1]) {
			sum += snapshot.ntc_temp_c[1];
			count++;
		}
#endif
		if (count > 0) {
			pwm_ch1_avg = sum / (float)count;
			pwm_ch1_valid = true;
		}

		sum = 0.0f;
		count = 0;
#if FEATURE_NTC_CH2_ENABLE
		if (snapshot.ntc_valid[2]) {
			sum += snapshot.ntc_temp_c[2];
			count++;
		}
#endif
#if FEATURE_NTC_CH3_ENABLE
		if (snapshot.ntc_valid[3]) {
			sum += snapshot.ntc_temp_c[3];
			count++;
		}
#endif
		if (count > 0) {
			pwm_ch0_avg = sum / (float)count;
			pwm_ch0_valid = true;
		}

		uint8_t payload_ntc[8] = {0};
		const size_t ntc_len = comm_protocol_pack_ntc_payload(
			pwm_ch0_valid,
			pwm_ch0_avg,
			pwm_ch1_valid,
			pwm_ch1_avg,
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
		// PID 输出帧（ms）。
		uint8_t payload_pid[8] = {0};
		size_t pid_len = comm_protocol_pack_pid_out_payload(
			snapshot.pwm_on_ms[1], payload_pid, sizeof(payload_pid));
		if (pid_len == 4) {
			const size_t pid_len2 = comm_protocol_pack_pid_out_payload(
				snapshot.pwm_on_ms[0], payload_pid + pid_len, sizeof(payload_pid) - pid_len);
			if (pid_len2 == 4) {
				pid_len += pid_len2;
			} else {
				pid_len = 0;
			}
		} else {
			pid_len = 0;
		}
		if (pid_len > 0) {
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
		const int len = comm_udp_receive_line(line, sizeof(line), 200);
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
                                                 