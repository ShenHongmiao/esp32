#include "ctrl_ntc.h"

#include <math.h>

#include "app_config.h"

bool ctrl_ntc_voltage_to_temp_c(float voltage_v, float *out_temp_c) {
    // 输出参数必须有效。
    if (out_temp_c == NULL) {
        return false;
    }

    // 分压电压必须在 (0, Vref) 开区间内，否则公式分母会异常。
    if (voltage_v <= 0.0f || voltage_v >= APP_ADC_VREF_V) {
        return false;
    }

    // 根据分压公式反推出 NTC 阻值：Rntc = Rs * V / (Vref - V)。
    const float r_ntc = APP_NTC_SERIES_RES_OHM * voltage_v / (APP_ADC_VREF_V - voltage_v);
    if (r_ntc <= 0.0f) {
        return false;
    }

    // Beta 模型：1/T = 1/T0 + (1/B) * ln(R/R0)
    const float t0_k = APP_NTC_T0_C + 273.15f;
    const float inv_t = (1.0f / t0_k) + (1.0f / APP_NTC_BETA) * logf(r_ntc / APP_NTC_R0_OHM);
    if (inv_t <= 0.0f) {
        return false;
    }

    // 开尔文转摄氏度。
    *out_temp_c = (1.0f / inv_t) - 273.15f;
    return true;
}
