#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ezapp.hpp"

#define BLINK_GPIO GPIO_NUM_2

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize EzApp singleton and demo typed read/writes (C++ API)
    EzApp &app = EzApp::instance();
    if (!app.init()) {
        ESP_LOGE("kc868", "EzApp init failed");
    } else {
        ESP_LOGI("kc868", "EzApp initialized");

        short s = 12345;
        app.writeShort(EzApp::X, 0, s);
        short s_out = 0;
        app.readShort(EzApp::X, 0, s_out);
        ESP_LOGI("kc868", "short read back: %d", s_out);

        int16_t i16 = -1234;
        app.writeInt16(EzApp::Y, 10, i16);
        int16_t i16_out = 0;
        app.readInt16(EzApp::Y, 10, i16_out);
        ESP_LOGI("kc868", "int16 read back: %d", i16_out);

        int32_t i32 = 0x12345678;
        app.writeInt32(EzApp::D, 100, i32);
        int32_t i32_out = 0;
        app.readInt32(EzApp::D, 100, i32_out);
        ESP_LOGI("kc868", "int32 read back: 0x%08x", (uint32_t)i32_out);

        float f = 3.14159f;
        app.writeFloat(EzApp::X, 200, f);
        float f_out = 0.0f;
        app.readFloat(EzApp::X, 200, f_out);
        ESP_LOGI("kc868", "float read back: %f", f_out);
    }

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);
        ESP_LOGI("kc868", "LED ON");
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BLINK_GPIO, 0);
        ESP_LOGI("kc868", "LED OFF");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
