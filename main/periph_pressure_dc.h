#ifndef PERIPH_PRESSURE_DC_H
#define PERIPH_PRESSURE_DC_H

#include <stdbool.h>

#include "esp_err.h"

typedef struct {
    float pressure_kpa;
    float voltage_v;
} periph_pressure_dc_sample_t;

// Read selected DC pressure channel and convert to kPa.
esp_err_t periph_pressure_dc_read(periph_pressure_dc_sample_t *out_sample);

// Convert sensor output voltage to kPa using Vout = slope * P + offset.
float periph_pressure_dc_voltage_to_kpa(float voltage_v);

#endif  // PERIPH_PRESSURE_DC_H
