#include "periph_pressure_dc.h"

#include "periph_adc.h"
#include "pressure_config.h"

float periph_pressure_dc_voltage_to_kpa(float voltage_v) {
    return (voltage_v - APP_PRESSURE_VOUT_OFFSET_V) / APP_PRESSURE_VOUT_SLOPE_V_PER_KPA;
}

esp_err_t periph_pressure_dc_read(periph_pressure_dc_sample_t *out_sample) {
    if (out_sample == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

#if !APP_PRESSURE_SOURCE_DC
    out_sample->pressure_kpa = 0.0f;
    out_sample->voltage_v = 0.0f;
    return ESP_ERR_INVALID_STATE;
#else
    uint16_t raw12 = 0;
    const esp_err_t err = periph_adc_read_raw12(APP_PRESSURE_DC_ADC_CMD, &raw12);
    if (err != ESP_OK) {
        return err;
    }

    const float adc_voltage_v = periph_adc_raw12_to_voltage(raw12, APP_ADC_VREF_V);
    const float sensor_voltage_v = adc_voltage_v * APP_PRESSURE_DC_VOUT_SCALE;
    float pressure_kpa = periph_pressure_dc_voltage_to_kpa(sensor_voltage_v);
    if (pressure_kpa < 0.0f) {
        pressure_kpa = 0.0f;
    }

    out_sample->pressure_kpa = pressure_kpa;
    out_sample->voltage_v = sensor_voltage_v;
    return ESP_OK;
#endif
}
