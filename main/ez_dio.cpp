#include "ez_dio.h"
#include "ezapp.h"
#include "dfrobot_oxygen_sensor.h"
#include "adc_mgr.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>

static int adc1_u1_channel_to_gpio(adc_channel_t ch)
{
    switch (ch) {
    case ADC_CHANNEL_0: return 36; // GPIO36
    case ADC_CHANNEL_1: return 37; // GPIO37 (usually not exposed)
    case ADC_CHANNEL_2: return 38; // GPIO38 (usually not exposed)
    case ADC_CHANNEL_3: return 39; // GPIO39
    case ADC_CHANNEL_4: return 32; // GPIO32
    case ADC_CHANNEL_5: return 33; // GPIO33
    case ADC_CHANNEL_6: return 34; // GPIO34
    case ADC_CHANNEL_7: return 35; // GPIO35
    default: return -1;
    }
}

static const char *TAG = "DIO";

#define EZ_TASK_STACK 4096
#define OXYGEN_SENSOR_ADDR 0x73

// Some SEN0322-compatible modules report the OXYGEN_DATA_REGISTER bytes in 0.1uA units.
// If you see O2 about 10x too low (e.g., ~2% in air), set this to 10.
#ifndef OXYGEN_CUR_MULTIPLIER
#define OXYGEN_CUR_MULTIPLIER 10.0f
#endif

// D-registers used for oxygen calibration control (byte offsets)
// D116: calibration command
//       - 0: idle
//       - 1: trigger air calibration (20.9%) (backward-compatible)
//       - N: trigger calibration with value N = O2%*10 (e.g. 209 for 20.9%)
// D117: status (0=idle, 1=ok, negative=error)
static constexpr uint32_t OXYGEN_CALIB_CMD_OFFSET = 116 * 2;
static constexpr uint32_t OXYGEN_CALIB_STATUS_OFFSET = 117 * 2;

// Some modules never expose/store the calibration key at 0x0A (always reads 0x0000).
// In that case, compute a software key once in air at boot and use it as a fallback.
static float s_oxygen_key_fallback = 0.0f;
static bool s_oxygen_key_fallback_valid = false;

static float s_o2_stats_min = 1e9f;
static float s_o2_stats_max = -1e9f;
static float s_cur_stats_min = 1e9f;
static float s_cur_stats_max = -1e9f;
static int s_o2_stats_count = 0;
static bool s_o2_dumped_once = false;

static adc_channel_t adc_ch1 = ADC_CHANNEL_0; // GPIO36 (ADC1)
static adc_channel_t adc_ch2 = ADC_CHANNEL_3; // GPIO39 (ADC1)
static adc_channel_t adc_ch3 = ADC_CHANNEL_6; // GPIO34 (ADC1)
static adc_channel_t adc_ch4 = ADC_CHANNEL_7; // GPIO35 (ADC1)

static bool i2c_probe_addr(i2c_port_t port, uint8_t addr_7bit)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr_7bit << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err == ESP_OK;
}

static void i2c_scan_bus(i2c_port_t port)
{
    char line[256];
    size_t used = 0;
    used += snprintf(line + used, sizeof(line) - used, "I2C scan:");
    for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
        if (i2c_probe_addr(port, addr)) {
            used += snprintf(line + used, sizeof(line) - used, " 0x%02X", addr);
            if (used >= sizeof(line) - 8) {
                ESP_LOGI(TAG, "%s", line);
                used = 0;
                used += snprintf(line + used, sizeof(line) - used, "I2C scan(cont):");
            }
        }
    }
    ESP_LOGI(TAG, "%s", line);
}

// Control logic extracted from main loop for readability and testability.
static int16_t ez_dio_control_idle_state(EzApp &app, int16_t y0)
{
    int16_t d131 = 0;
    app.readInt16(EzApp::D, 131 * 2, d131);
    //온도제어
    if (d131 & 0x0004) { // D131.2 == 1
        y0 |= 0x0010;  // Y0.4 = 1
        app.writeInt16(EzApp::D, 109 * 2, 2);
    } else if (d131 & 0x0008) { // D131.3 == 1
        y0 &= ~0x0010; // Y0.4 = 0
    }
    //압력제어
    if (d131 & 0x0010) { // D131.4 == 1
        y0 |= 0x0004;  // Y0.2 = 1
        app.writeInt16(EzApp::D, 109 * 2, 4);
    } else if (d131 & 0x0020) { // D131.5 == 1
        y0 &= ~0x0004; // Y0.2 = 0
    }
    //배출제어
    if (d131 & 0x0040) { // D131.6 == 1
        y0 |= 0x0008;  // Y0.3 = 1
        app.writeInt16(EzApp::D, 109 * 2, 8);
        y0 &= ~0x0008; // Y0.3 = 0
    }

    //산소센서제어
    int16_t d132 = 0;
    app.readInt16(EzApp::D, 132 * 2, d132);
    if (d132 & 0x0001) { // D132.0 == 1
        app.writeInt16(EzApp::D, 109 * 2, 16);
        y0 |= 0x0020;  // Y0.5 = 1
    } else if (d132 & 0x0002) { // D132.1 == 1
        y0 &= ~0x0020; // Y0.5 = 0
    }

    if (d132 & 0x0004) { // D132.2 == 1
        app.writeInt16(EzApp::D, 109 * 2, 32);
        y0 &= ~0x0002; // Y0.1 = 0
        y0 |= 0x0001;  // Y0.0 = 1
        app.writeInt16(EzApp::D, 109 * 2, 64);
    } else if (d132 & 0x0008) { // D132.3 == 1
        y0 |= 0x0002;  // Y0.1 = 1
        y0 &= ~0x0001; // Y0.0 = 0
    }

    // 치료시작
    if (d131 & 0x0001) { // D131.0 == 1
        app.writeInt16(EzApp::D, 102 * 2, 10);
    }

    return y0;
}

static int16_t ez_dio_control_startup_state(EzApp &app, int16_t d102, int16_t y0)
{
    if (d102 == 10) {
        y0 &= ~0x0001; // Y0.0 = 0
        y0 &= ~0x0002; // Y0.1 = 0
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0
        static uint32_t start_time = 0;
        if (start_time == 0) {
            start_time = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(500)) {
            app.writeInt16(EzApp::D, 102 * 2, 20);
            start_time = 0;  // reset for next cycle
        }
        // Initialize D109
        app.writeInt16(EzApp::D, 109 * 2, 1);
    } else if (d102 == 20) {
        app.writeInt16(EzApp::D, 109 * 2, 2);
        static uint32_t start_time_20 = 0;
        if (start_time_20 == 0) {
            start_time_20 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(2000)) {
            app.writeInt16(EzApp::D, 102 * 2, 30);
            start_time_20 = 0;  // reset for next cycle
        }
    } else if (d102 == 30) {
        app.writeInt16(EzApp::D, 109 * 2, 4);
        static uint32_t start_time_21 = 0;
        if (start_time_21 == 0) {
            start_time_21 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_21) >= pdMS_TO_TICKS(2000)) {
            app.writeInt16(EzApp::D, 102 * 2, 22);
            start_time_21 = 0;  // reset for next cycle
        }
    }

    return y0;
}

static void ez_dio_task(void *arg)
{

    ESP_LOGI(TAG, "ADC(U1) channels: ch1=%d(GPIO%d), ch2=%d(GPIO%d), ch3=%d(GPIO%d), ch4=%d(GPIO%d)",
             adc_ch1, adc1_u1_channel_to_gpio(adc_ch1),
             adc_ch2, adc1_u1_channel_to_gpio(adc_ch2),
             adc_ch3, adc1_u1_channel_to_gpio(adc_ch3),
             adc_ch4, adc1_u1_channel_to_gpio(adc_ch4));
    (void)arg;
    EzApp &app = EzApp::instance();
    int oxygen_counter = 0;  // counter for oxygen sensor reading

    DfRobotOxygenSensor oxygen;
    oxygen.setCurrentMultiplier((float)OXYGEN_CUR_MULTIPLIER);
    bool oxygen_ok = false;

    // Auto-calibrate oxygen sensor once at boot (user request)
    {
        // Basic sanity: scan I2C bus.
        i2c_scan_bus(static_cast<i2c_port_t>(EzApp::I2C_PORT));
        if (!i2c_probe_addr(static_cast<i2c_port_t>(EzApp::I2C_PORT), OXYGEN_SENSOR_ADDR)) {
            ESP_LOGW(TAG, "O2 sensor addr 0x%02X not found on I2C", OXYGEN_SENSOR_ADDR);
        } else {
            esp_err_t b = oxygen.begin(static_cast<i2c_port_t>(EzApp::I2C_PORT), OXYGEN_SENSOR_ADDR);
            if (b == ESP_OK) {
                oxygen_ok = true;
            } else {
                ESP_LOGW(TAG, "O2 begin failed: %d", b);
            }
        }
        // give system/i2c time to settle
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
    // Initialize shared ADC manager once (safe to call repeatedly).
    esp_err_t adc_init_err = adc_mgr_init();
    if (adc_init_err != ESP_OK) {
        ESP_LOGE(TAG, "adc_mgr_init failed: %d", adc_init_err);
    }
        int loop_cnt = 0;
    while (1) {
        // Check D102; if 0, control Y0 bits 3-4 based on D132 bits 2-3
        int16_t d102 = 0;
        app.readInt16(EzApp::D, 102 * 2, d102);
        int16_t y0 = 0;
        app.readInt16(EzApp::Y, 0, y0);

        if (d102 == 0) {
            ez_dio_control_idle_state(app, y0);
        } else {
            ez_dio_control_startup_state(app, d102, y0);
        }


        // 1) Increment D group at offset 0 (int32)
        app.writeInt32(EzApp::D, 0, loop_cnt);


        // 2) Read PCF8574 inputs and store to X group offset 0
        uint8_t in_byte = 0;
        esp_err_t r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_INPUT_ADDR, &in_byte, 1, pdMS_TO_TICKS(1000));
        if (r == ESP_OK) {
            app.writeInt16(EzApp::X, 0, static_cast<int16_t>(in_byte));
            // record DI (inputs) to D offset 100
            app.writeInt16(EzApp::D, 100 * 2, static_cast<int16_t>(in_byte));
        } else {
            ESP_LOGW(TAG, "Failed read PCF_INPUT: %d", r);
        }

        // 3) Read Y group offset 0 and write to PCF8574 outputs



        // write updated Y back so other tasks see the forced state
        app.writeInt16(EzApp::Y, 0, y0);
        uint8_t out_byte = static_cast<uint8_t>(y0 & 0xFF);
        r = i2c_master_write_to_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_OUTPUT_ADDR, &out_byte, 1, pdMS_TO_TICKS(1000));
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "Failed write PCF_OUTPUT: %d", r);
        }
        // record DO (outputs) to D offset 101
        app.writeInt16(EzApp::D, 101 * 2, static_cast<int16_t>(out_byte));

        // 4) Read oxygen sensor and store to D114
        //if (oxygen_ok) 
        {
            oxygen_counter++;
            if (oxygen_counter >= 50) {  // read every ~100ms (50 * 2ms)
                oxygen_counter = 0;
                float o2_value = 0.0f;
                esp_err_t oer = oxygen.getOxygenData(10 /*collectNum*/, &o2_value);
                if (oer == ESP_OK) {
                    int16_t o2_int = static_cast<int16_t>(o2_value * 10);  // scale to 0.1% units
                    app.writeInt16(EzApp::D, 114 * 2, o2_int);
                } else {
                    ESP_LOGW(TAG, "O2 read failed: %d", oer);
                }
            }
        }


        loop_cnt++;
        // Short delay to allow RTOS scheduling and I2C/ADC activity.
        // Use 2ms as the shortest reasonable loop considering I2C/ADC access
        // (oxygen sampling logic expects ~2ms loop: 50 * 2ms => ~100ms sampling).
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
    // Test is currently disabled (body commented out)
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
