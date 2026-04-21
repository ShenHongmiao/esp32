#include "periph_pwm.h"

#include "driver/ledc.h"

#include "app_config.h"

// 保存最后一次设置值（ms），便于遥测和调试读取。
static float s_last_on_time_ms = 0.0f;

#ifndef APP_PWM_PERIOD_MS
#define APP_PWM_PERIOD_MS 1000.0f
#endif

#ifndef APP_PWM_CH0_ENABLE
#define APP_PWM_CH0_ENABLE 1
#endif

#ifndef APP_PWM_CH1_ENABLE
#define APP_PWM_CH1_ENABLE 1
#endif

static void pwm_apply_percent(float duty_percent) {
    // 占空比限幅，避免异常输入导致驱动越界。
    if (duty_percent < 0.0f) {
        duty_percent = 0.0f;
    }
    if (duty_percent > 100.0f) {
        duty_percent = 100.0f;
    }

    // 将百分比映射到 LEDC 计数值（10bit 对应 0~1023）。
    const uint32_t max_duty = (1u << LEDC_TIMER_10_BIT) - 1u;
    const uint32_t duty = (uint32_t)((duty_percent / 100.0f) * (float)max_duty);

    // 仅对配置中使能的通道下发占空比。
#if APP_PWM_CH0_ENABLE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
#endif

#if APP_PWM_CH1_ENABLE
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
#endif
}

void periph_pwm_set_on_time_ms(float on_time_ms) {
    if (on_time_ms < 0.0f) {
        on_time_ms = 0.0f;
    }
    if (on_time_ms > APP_PWM_PERIOD_MS) {
        on_time_ms = APP_PWM_PERIOD_MS;
    }

    const float duty_percent = (on_time_ms / APP_PWM_PERIOD_MS) * 100.0f;
    pwm_apply_percent(duty_percent);
    s_last_on_time_ms = on_time_ms;
}

esp_err_t periph_pwm_init(void) {
    // 配置 PWM 定时器：频率来自统一配置，分辨率 10bit。
    const ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = APP_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        return err;
    }

    // 通道 0 绑定 GPIO4。
#if APP_PWM_CH0_ENABLE
    const ledc_channel_config_t ch0 = {
        .gpio_num = APP_PWM_GPIO_CH0,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
#endif

    // 通道 1 绑定 GPIO5。
#if APP_PWM_CH1_ENABLE
    const ledc_channel_config_t ch1 = {
        .gpio_num = APP_PWM_GPIO_CH1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
#endif

#if APP_PWM_CH0_ENABLE
    err = ledc_channel_config(&ch0);
    if (err != ESP_OK) {
        return err;
    }
#endif

#if APP_PWM_CH1_ENABLE
    err = ledc_channel_config(&ch1);
    if (err != ESP_OK) {
        return err;
    }
#endif

    // 上电默认关断输出，避免瞬态加热。
    periph_pwm_set_on_time_ms(0.0f);
    return ESP_OK;
}

void periph_pwm_set_percent(float duty_percent) {
    if (duty_percent < 0.0f) {
        duty_percent = 0.0f;
    }
    if (duty_percent > 100.0f) {
        duty_percent = 100.0f;
    }

    pwm_apply_percent(duty_percent);
    s_last_on_time_ms = (duty_percent / 100.0f) * APP_PWM_PERIOD_MS;
}

void periph_pwm_force_off(void) {
    periph_pwm_set_on_time_ms(0.0f);
}

float periph_pwm_get_on_time_ms(void) {
    return s_last_on_time_ms;
}

float periph_pwm_get_percent(void) {
    return (s_last_on_time_ms / APP_PWM_PERIOD_MS) * 100.0f;
}
