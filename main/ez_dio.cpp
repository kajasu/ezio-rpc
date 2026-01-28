#include "ez_dio.h"
#include "ezapp.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define EZ_TASK_STACK 4096

static void ez_dio_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    while (1) {
        // 1) Increment D group at offset 0 (int32)
        int32_t d = 0;
        app.readInt32(EzApp::D, 0, d);
        d += 1;
        app.writeInt32(EzApp::D, 0, d);

        // 2) Read PCF8574 inputs and store to X group offset 0
        uint8_t in_byte = 0;
        esp_err_t r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_INPUT_ADDR, &in_byte, 1, pdMS_TO_TICKS(1000));
        if (r == ESP_OK) {
            app.writeInt16(EzApp::X, 0, static_cast<int16_t>(in_byte));
            // record DI (inputs) to D offset 525
            app.writeInt16(EzApp::D, 525, static_cast<int16_t>(in_byte));
        } else {
            ESP_LOGW("kc868", "Failed read PCF_INPUT: %d", r);
        }

        // 3) Read Y group offset 0 and write to PCF8574 outputs
        int16_t y = 0;
        app.readInt16(EzApp::Y, 0, y);
        uint8_t out_byte = static_cast<uint8_t>(y & 0xFF);
        r = i2c_master_write_to_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_OUTPUT_ADDR, &out_byte, 1, pdMS_TO_TICKS(1000));
        if (r != ESP_OK) {
            ESP_LOGW("kc868", "Failed write PCF_OUTPUT: %d", r);
        }
        // record DO (outputs) to D offset 526
        app.writeInt16(EzApp::D, 526, static_cast<int16_t>(out_byte));
        
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelete(NULL);
}

void start_ez_dio_task()
{
    xTaskCreate(ez_dio_task, "ez_dio_task", EZ_TASK_STACK, NULL, 5, NULL);
}
