#pragma once

#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes ADC oneshot unit (ADC_UNIT_1) once. Safe to call multiple times.
esp_err_t adc_mgr_init(void);

// Ensures the given channel is configured (12dB, default bitwidth) and reads a single raw sample.
esp_err_t adc_mgr_read(adc_channel_t channel, int *out_raw);

#ifdef __cplusplus
}
#endif
