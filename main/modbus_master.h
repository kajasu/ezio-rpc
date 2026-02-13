#pragma once
#include <stdint.h>
#include "esp_err.h"

void start_modbus_master_task();
// Persist/load helper for D130..D159 region
esp_err_t save_d130_region_to_nvs();
esp_err_t load_d130_region_from_nvs();
