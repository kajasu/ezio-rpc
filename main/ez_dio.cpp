#include "ez_dio.h"
#include "ezapp.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>

#define EZ_TASK_STACK 4096
#define OXYGEN_SENSOR_ADDR 0x72

static void ez_dio_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    int oxygen_counter = 0;  // counter for oxygen sensor reading
    while (1) {
        
        
        
        // Check D102; if 0, control Y0 bits 3-4 based on D132 bits 2-3
        int16_t d102 = 0;
        app.readInt16(EzApp::D, 102 * 2, d102);
        if (d102 == 0) {
            int16_t d132 = 0;
            app.readInt16(EzApp::D, 132 * 2, d132);
            int16_t y0 = 0;
            app.readInt16(EzApp::Y, 0, y0);
            
            if (d132 & 0x0004) { // D132.2 == 1
                y0 |= 0x0008;   // Y0.3 = 1
                y0 &= ~0x0010;  // Y0.4 = 0
            } else if (d132 & 0x0008) { // D132.3 == 1
                y0 &= ~0x0008;  // Y0.3 = 0
                y0 |= 0x0010;   // Y0.4 = 1
            }
            app.writeInt16(EzApp::Y, 0, y0);
        }










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

        // Check D257 (byte-offset = 257*2 = 514) bits:
        // - If bit1 == 1 -> clear Y0 (higher priority)
        // - Else if bit0 == 1 -> set Y0
        int16_t d257 = 0;
        app.readInt16(EzApp::D, 257 * 2, d257);
        if (d257 & 0x0002) {
            y &= static_cast<int16_t>(~0x0001); // clear Y0
        } else if (d257 & 0x0001) {
            y |= 0x0001; // set Y0
        }

        // write updated Y back so other tasks see the forced state
        app.writeInt16(EzApp::Y, 0, y);

        uint8_t out_byte = static_cast<uint8_t>(y & 0xFF);
        r = i2c_master_write_to_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_OUTPUT_ADDR, &out_byte, 1, pdMS_TO_TICKS(1000));
        if (r != ESP_OK) {
            ESP_LOGW("kc868", "Failed write PCF_OUTPUT: %d", r);
        }
        // record DO (outputs) to D offset 526
        app.writeInt16(EzApp::D, 526, static_cast<int16_t>(out_byte));
        
        // Read oxygen sensor every ~1 second (500 loops * 2ms = 1000ms)
        oxygen_counter++;
        if (oxygen_counter >= 500) {
            oxygen_counter = 0;
            uint8_t o2_buf[4];
            esp_err_t o2_r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), 
                                                          OXYGEN_SENSOR_ADDR, o2_buf, sizeof(o2_buf), 
                                                          pdMS_TO_TICKS(500));
            if (o2_r == ESP_OK) {
                float val;
                memcpy(&val, o2_buf, sizeof(val));
                int32_t scaled = (int32_t)(val * 100.0f);
                if (scaled < 0) scaled = 0;
                if (scaled > 0xFFFF) scaled = 0xFFFF;
                app.writeInt16(EzApp::D, EzApp::OXYGEN_OFFSET1, (int16_t)scaled);
            } else {
                ESP_LOGW("kc868", "O2 sensor read failed: %d", o2_r);
            }
        }
        
        // Periodically print 6 int16 values from D starting at D256 (byte offset = 256*2)
        static int loop_cnt = 0;
        loop_cnt++;
        // if ((loop_cnt % 5) == 0) { // ~1s (5 * 200ms)
        //     const uint32_t base = 256 * 2; // byte offset
        //     int16_t vals[6];
        //     for (int i = 0; i < 6; ++i) {
        //         vals[i] = 0;
        //         app.readInt16(EzApp::D, base + i * 2, vals[i]);
        //     }
        //     ESP_LOGI("kc868", "D[256*2..] 6 vals: %d, %d, %d, %d, %d, %d",
        //              vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
        // }



        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelete(NULL);
}

void start_ez_dio_task()
{
    xTaskCreate(ez_dio_task, "ez_dio_task", EZ_TASK_STACK, NULL, 5, NULL);
}

// Y0 toggle test: cycles bits 0..7 of Y (offset 0) one bit at a time every 1 second.
static void y0_toggle_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    int bit = 0;
    while (1) {
        // int16_t v = static_cast<int16_t>(1 << bit);
        // app.writeInt16(EzApp::Y, 0, v);
        // bit = (bit + 1) & 0x7; // cycle 0..7
        // int16_t y_cur = 0;
        // app.readInt16(EzApp::Y, 0, y_cur);
        // ESP_LOGI("kc868", "Y0 toggle test set: 0x%02X", static_cast<unsigned int>(y_cur & 0xFF));
        


        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelete(NULL);
}

void start_y0_toggle_test()
{
    xTaskCreate(y0_toggle_task, "y0_toggle", 2048, NULL, 5, NULL);
}
