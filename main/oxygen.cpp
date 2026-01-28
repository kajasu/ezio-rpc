#include "oxygen.h"
#include "ezapp.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static void oxygen_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    const uint8_t addr = 0x72;
    uint8_t buf[4];
    while (1) {
        int r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), addr, buf, sizeof(buf), pdMS_TO_TICKS(500));
        if (r == (int)sizeof(buf)) {
            float val;
            memcpy(&val, buf, sizeof(val));
            int32_t scaled = (int32_t)(val * 100.0f);
            if (scaled < 0) scaled = 0;
            if (scaled > 0xFFFF) scaled = 0xFFFF;
            app.writeInt16(EzApp::D, EzApp::OXYGEN_OFFSET1, (int16_t)scaled);
        } else {
            ESP_LOGW("kc868", "O2 read1 failed: %d", r);
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), addr, buf, sizeof(buf), pdMS_TO_TICKS(500));
        if (r == (int)sizeof(buf)) {
            float val;
            memcpy(&val, buf, sizeof(val));
            int32_t scaled = (int32_t)(val * 100.0f);
            if (scaled < 0) scaled = 0;
            if (scaled > 0xFFFF) scaled = 0xFFFF;
            app.writeInt16(EzApp::D, EzApp::OXYGEN_OFFSET2, (int16_t)scaled);
        } else {
            ESP_LOGW("kc868", "O2 read2 failed: %d", r);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

void start_oxygen_task()
{
    xTaskCreate(oxygen_task, "oxygen", 4096, NULL, 5, NULL);
}
