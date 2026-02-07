#include "adc_mgr.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "adc_mgr";

static adc_oneshot_unit_handle_t s_handle = nullptr;
static SemaphoreHandle_t s_mutex = nullptr;
static uint32_t s_configured_mask = 0; // bit per adc_channel_t (0..9 depending on target)

static inline uint32_t channel_bit(adc_channel_t ch)
{
    if (ch < 0) return 0;
    if (ch >= 32) return 0;
    return 1u << static_cast<uint32_t>(ch);
}

esp_err_t adc_mgr_init(void)
{
    if (s_handle != nullptr) return ESP_OK;

    if (s_mutex == nullptr) {
        s_mutex = xSemaphoreCreateMutex();
        if (s_mutex == nullptr) return ESP_ERR_NO_MEM;
    }

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (s_handle == nullptr) {
        adc_oneshot_unit_init_cfg_t init_cfg{};
        init_cfg.unit_id = ADC_UNIT_1;
        init_cfg.ulp_mode = ADC_ULP_MODE_DISABLE;

        esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %d", err);
            xSemaphoreGive(s_mutex);
            return err;
        }
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t adc_mgr_read(adc_channel_t channel, int *out_raw)
{
    if (out_raw == nullptr) return ESP_ERR_INVALID_ARG;

    esp_err_t err = adc_mgr_init();
    if (err != ESP_OK) return err;

    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t bit = channel_bit(channel);
    if ((s_configured_mask & bit) == 0) {
        adc_oneshot_chan_cfg_t chan_cfg{};
        chan_cfg.atten = ADC_ATTEN_DB_12;
        chan_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;

        err = adc_oneshot_config_channel(s_handle, channel, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "adc_oneshot_config_channel(%d) failed: %d", (int)channel, err);
            xSemaphoreGive(s_mutex);
            return err;
        }
        s_configured_mask |= bit;
    }

    err = adc_oneshot_read(s_handle, channel, out_raw);
    xSemaphoreGive(s_mutex);
    return err;
}
