#include "ez_dio.h"
#include "ezapp.h"
#include "adc_mgr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "DFRobot_EOxygenSensor.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
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

// Offset for voice output number (D-register byte offset)
static constexpr int voice_out_no_addr = 109 * 2;
// Address for curing step D-register (byte offset)
static constexpr int curing_step_addr = 102 * 2;



static adc_channel_t adc_ch1 = ADC_CHANNEL_0; // GPIO36 (ADC1)
static adc_channel_t adc_ch2 = ADC_CHANNEL_3; // GPIO39 (ADC1)
static adc_channel_t adc_ch3 = ADC_CHANNEL_6; // GPIO34 (ADC1)
static adc_channel_t adc_ch4 = ADC_CHANNEL_7; // GPIO35 (ADC1)

// I2C helper functions removed (oxygen sensor code deleted)

// I2C bus scanner: scans all possible 7-bit addresses and reports found devices
static void i2c_scan_bus(i2c_port_t port, const char *port_name)
{
    ESP_LOGI(TAG, "Scanning I2C bus on %s...", port_name);
    int devices_found = 0;
    
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "  No I2C devices found on %s", port_name);
    } else {
        ESP_LOGI(TAG, "  Total %d device(s) found on %s", devices_found, port_name);
    }
}

// Oxygen sensor task: reads E-Oxygen sensor and writes to D114
static void oxygen_sensor_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    
    // Configure separate I2C port for oxygen sensor (I2C_NUM_1)
    // GPIO 5 = SCL, GPIO 22 = SDA
    const i2c_port_t OXYGEN_I2C_PORT = I2C_NUM_1;
    const int OXYGEN_SDA_IO = 22;
    const int OXYGEN_SCL_IO = 5;
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = OXYGEN_SDA_IO;
    conf.scl_io_num = OXYGEN_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // 100kHz for oxygen sensor
    
    esp_err_t i2c_err = i2c_param_config(OXYGEN_I2C_PORT, &conf);
    if (i2c_err == ESP_OK) {
        i2c_err = i2c_driver_install(OXYGEN_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C for oxygen sensor: %d", i2c_err);
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG, "Oxygen sensor I2C initialized on port %d (SDA=%d, SCL=%d)", 
             OXYGEN_I2C_PORT, OXYGEN_SDA_IO, OXYGEN_SCL_IO);
    
    // Scan I2C bus to find devices
    i2c_scan_bus(OXYGEN_I2C_PORT, "I2C_NUM_1 (Oxygen)");
    
    // Initialize oxygen sensor on dedicated I2C port with address 0x70
    DFRobot_EOxygenSensor_I2C oxygen(OXYGEN_I2C_PORT, E_OXYGEN_ADDRESS_3);
    
    // Try to connect to sensor
    esp_err_t err = oxygen.begin();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "E-Oxygen sensor not found, task will retry...");
    }
    
    // Give sensor time to stabilize
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    while (1) {
        // Read oxygen concentration (returns % VOL)
        float o2_percent = oxygen.readOxygenConcentration();
        
        // Convert to int16 (scaled by 10 for 0.1% precision)
        // e.g., 20.9% becomes 209
        int16_t o2_val = static_cast<int16_t>(o2_percent * 100.0f);
        
        // Write to D114 register
        app.writeInt16(EzApp::D, 114 * 2, o2_val);
        ESP_LOGI(TAG, "O2 : %.2f", o2_val);
        // Log periodically (every 10 readings)
        static int log_counter = 0;
        if (++log_counter >= 10) {
            ESP_LOGI(TAG, "O2 concentration: %.2f%% VOL (D114=%d)", o2_percent, o2_val);
            log_counter = 0;
        }
        
        // Read every 5 seconds to reduce I2C bus usage
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    vTaskDelete(NULL);
}

// Control logic extracted from main loop for readability and testability.
static int16_t ez_dio_control_idle_state(EzApp &app, int16_t d131 , int16_t d132, int16_t x0, int16_t y0)
{

    // 문열기/닫기: 우선 음성(voice_out)만 재생하고 3초 후 실제 Y0 출력 적용
    static TickType_t door_deadline = 0;
    static int door_pending = 0; // 0: none, 1: close, 2: open
    static int prev_d132 = 0;

    // Detect new close command (rising edge D132.2)
    if ((d132 & 0x0004) && !(prev_d132 & 0x0004)) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 11); // voice: 닫기
        door_pending = 1;
        door_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    }

    // Detect new open command (rising edge D132.3)
    if ((d132 & 0x0008) && !(prev_d132 & 0x0008)) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 10); // voice: 열기
        door_pending = 2;
        door_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(3000);
    }

    // If a pending door action's deadline passed, apply the Y0 changes
    if (door_pending == 1 && (xTaskGetTickCount() >= door_deadline)) {
        // close: Y0.0 = 1, Y0.1 = 0
        y0 |= 0x0001;
        y0 &= ~0x0002;
        door_pending = 0;
    } else if (door_pending == 2 && (xTaskGetTickCount() >= door_deadline)) {
        // open: Y0.0 = 0, Y0.1 = 1
        y0 &= ~0x0001;
        y0 |= 0x0002;
        door_pending = 0;
    }

    prev_d132 = d132;

    //압력제어
    if (d131 & 0x0010) { // D131.4 == 1 
        app.writeInt16(EzApp::D, voice_out_no_addr, 12);
        y0 |= 0x0004;  // Y0.2 = 1
    } else if (d131 & 0x0020) { // D131. 5 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 13);
        y0 &= ~0x0004; // Y0.2 = 0
    }

    //배출제어
    if (d131 & 0x0040) { // D131.6 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 14);
        y0 |= 0x0008;  // Y0.1 = 1
    } else if (d131 & 0x0080) { // D131.7 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 15);
        y0 &= ~0x0008; // Y0.1 = 0  
    }

    //온도제어
    if (d131 & 0x0004) { // D131.2 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 16);
        y0 |= 0x0010;  // Y0.0 = 1
    } else if (d131 & 0x0008) { // D131.3 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 17);
        y0 &= ~0x0010; // Y0.0 = 0
    }

    //산소공급
    if (d132 & 0x0001) { // D132.0 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 18);
        y0 |= 0x0020;  // Y0.5 = 1
    } else if (d132 & 0x0002) { // D132.1 == 1
        app.writeInt16(EzApp::D, voice_out_no_addr, 19);
        y0 &= ~0x0020; // Y0.5 = 0
    }

    // 치료시작
    if ((d131 & 0x0001) || (x0 & 0x0001)) { // D131.0 == 1
        app.writeInt16(EzApp::D, curing_step_addr, 10);
    }

    return y0;
}

static int16_t ez_dio_control_startup_state(EzApp &app, int16_t d102,int16_t d131 ,int16_t x0, int16_t y0)
{

    // timing variables for startup state steps
    static uint32_t start_time = 0;
    static uint32_t start_time_20 = 0;
    static uint32_t start_time_30 = 0;
    static uint32_t start_time_40 = 0;
    static uint32_t start_time_50 = 0;
    static uint32_t start_time_80 = 0;

    // 치료시작
    if ((d131 & 0x0002)||(x0 & 0x0002)) { // D131.1 == 1
        app.writeInt16(EzApp::D, curing_step_addr, 0); //
        app.writeInt16(EzApp::D, voice_out_no_addr, 8); //치료가 중지합니다.
        start_time = 0;  // reset for next cycle
        start_time_20 = 0;  // reset for next cycle
        start_time_30 = 0;  // reset for next cycle
        start_time_40 = 0;  // reset for next cycle
        start_time_50 = 0;  // reset for next cycle
        start_time_80 = 0;  // reset for next cycle

    }
    //문닫고 ->  가압 -> 치료 ->  배출 -> 문열기    
    if (d102 == 10) {
        y0 &= ~0x0001; // Y0.0 = 0
        y0 &= ~0x0002; // Y0.1 = 0
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0
        start_time_20 = 0;  // reset for next cycle
        start_time_30 = 0;  // reset for next cycle
        start_time_40 = 0;  // reset for next cycle
        start_time_50 = 0;  // reset for next cycle
        start_time_80 = 0;  // reset for next cycle

        if (start_time == 0) {
            start_time = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(3000)) {
            app.writeInt16(EzApp::D, curing_step_addr, 20);
            start_time = 0;  // reset for next cycle
        }
        app.writeInt16(EzApp::D, voice_out_no_addr, 1);
    } else if (d102 == 20) {//문열기
        app.writeInt16(EzApp::D, voice_out_no_addr, 11); //잠시후 문이 닫힘니다
        if (start_time_20 == 0) {
            start_time_20 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(4000)) { // 대기 후 문닫기
            y0 |= 0x0001;  // Y0.1 = 1
            y0 &= ~0x0002; // Y0.0 = 0
        }
        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(10000)) { // 문닫고 다음으로
            app.writeInt16(EzApp::D, curing_step_addr, 30);
            start_time_20 = 0;  // reset for next cycle
        }
    } else if (d102 == 30) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 12); //압력 제어를 시작합니다.
        if (start_time_30 == 0) {
            start_time_30 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_30) >= pdMS_TO_TICKS(4000)) {
            y0 |= 0x0004;  // Y0.2 = 1
            app.writeInt16(EzApp::D, voice_out_no_addr, 16); //온도 제어를 시작합니다.
        }
        if ((xTaskGetTickCount() - start_time_30) >= pdMS_TO_TICKS(8000)) {
            y0 |= 0x0010;  // Y0.4 = 1
            app.writeInt16(EzApp::D, curing_step_addr, 40);
            start_time_30 = 0;  // reset for next cycle
        }

    }else if (d102 == 40) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 18); //산소공급을 시작합니다.
        if (start_time_40 == 0) {
            start_time_40 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_40) >= pdMS_TO_TICKS(2000)) {
            y0 |= 0x0020;  // Y0.0 = 1            
            app.writeInt16(EzApp::D, curing_step_addr, 50);
            start_time_40 = 0;  // reset for next cycle
        }
    }else if (d102 == 50) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 14); //배출을 시작합니다.
        if (start_time_50 == 0) {
            start_time_50 = xTaskGetTickCount();
        }
        if ((xTaskGetTickCount() - start_time_50) >= pdMS_TO_TICKS(2000)) {
            y0 |= 0x0008;  // Y0.0 = 1

            app.writeInt16(EzApp::D, curing_step_addr, 80);
            start_time_50 = 0;  // reset for next cycle
        }
    }else if (d102 == 80) {
        if (start_time_80 == 0) {
            start_time_80 = xTaskGetTickCount();
        }   
        app.writeInt16(EzApp::D, voice_out_no_addr, 10); //잠시후 문이 열립니다
        if ((xTaskGetTickCount() - start_time_80) >= pdMS_TO_TICKS(3000)) {

            y0 |= 0x0002;  // Y0.1 = 1
            y0 &= ~0x0001; // Y0.0 = 0

            app.writeInt16(EzApp::D, curing_step_addr, 99);
            start_time_80 = 0;  // reset for next cycle
        }

    }else if (d102 == 99) {
        //y0 &= ~0x0001; // Y0.0 = 0
        //y0 &= ~0x0002; // Y0.1 = 0
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0

        app.writeInt16(EzApp::D, curing_step_addr, 0); //
        app.writeInt16(EzApp::D, voice_out_no_addr, 8); //치료가 중지합니다.
    }
    return y0;
}

// Oxygen sensor task removed.

static void ez_dio_task(void *arg)
{

    // ESP_LOGI(TAG, "ADC(U1) channels: ch1=%d(GPIO%d), ch2=%d(GPIO%d), ch3=%d(GPIO%d), ch4=%d(GPIO%d)",
    //          adc_ch1, adc1_u1_channel_to_gpio(adc_ch1),
    //          adc_ch2, adc1_u1_channel_to_gpio(adc_ch2),
    //          adc_ch3, adc1_u1_channel_to_gpio(adc_ch3),
    //          adc_ch4, adc1_u1_channel_to_gpio(adc_ch4));
    (void)arg;
    EzApp &app = EzApp::instance();
    // Loop timing stats (microseconds)
    static uint64_t loop_sum_us = 0;
    static uint32_t loop_min_us = UINT32_MAX;
    static uint32_t loop_max_us = 0;
    static uint32_t loop_count = 0;
    static int64_t last_report_time_us = 0;
    // Per-section timing stats (A..E) in microseconds
    // A: D increment
    static uint64_t secA_sum_us = 0; static uint32_t secA_min_us = UINT32_MAX; static uint32_t secA_max_us = 0; static uint32_t secA_count = 0;
    // B: PCF8574 read + X/D100 writes
    static uint64_t secB_sum_us = 0; static uint32_t secB_min_us = UINT32_MAX; static uint32_t secB_max_us = 0; static uint32_t secB_count = 0;
    // C: read D group + control logic
    static uint64_t secC_sum_us = 0; static uint32_t secC_min_us = UINT32_MAX; static uint32_t secC_max_us = 0; static uint32_t secC_count = 0;
    // D: Y write + PCF8574 output write + D101
    static uint64_t secD_sum_us = 0; static uint32_t secD_min_us = UINT32_MAX; static uint32_t secD_max_us = 0; static uint32_t secD_count = 0;
    // Note: oxygen reads moved to separate task; per-loop 'E' stats removed.

    // Oxygen sensor reads are handled in a separate task started below.
    // Initialize shared ADC manager once (safe to call repeatedly).
    esp_err_t adc_init_err = adc_mgr_init();
    if (adc_init_err != ESP_OK) {
        ESP_LOGE(TAG, "adc_mgr_init failed: %d", adc_init_err);
    }
    
    // Scan I2C_NUM_0 bus for PCF8574 and other devices
    static bool i2c_scanned = false;
    if (!i2c_scanned) {
        i2c_scan_bus(static_cast<i2c_port_t>(EzApp::I2C_PORT), "I2C_NUM_0 (PCF8574)");
        i2c_scanned = true;
    }
    
    // Start oxygen sensor task to read E-Oxygen sensor
    static bool oxygen_task_started = false;
    if (!oxygen_task_started) {
        xTaskCreate(oxygen_sensor_task, "oxygen_sensor", 4096, NULL, 4, NULL);
        oxygen_task_started = true;
        ESP_LOGI(TAG, "E-Oxygen sensor task started");
    }
    
    int loop_cnt = 0;
    int16_t x0 = 0;
    while (1) {
            int64_t loop_start_us = esp_timer_get_time();
        // 1) Increment D group at offset 0 (int32)
        app.writeInt32(EzApp::D, 0, loop_cnt++);
        int64_t t_after_A = esp_timer_get_time();
        uint32_t durA = (uint32_t)(t_after_A - loop_start_us);
        secA_sum_us += durA; if (durA < secA_min_us) secA_min_us = durA; if (durA > secA_max_us) secA_max_us = durA; secA_count++;

        // 2) Read PCF8574 inputs and store to X group offset 0 (with retry)
        uint8_t raw = 0;
        esp_err_t r = ESP_FAIL;
        for (int retry = 0; retry < 3 && r != ESP_OK; retry++) {
            r = i2c_master_read_from_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_INPUT_ADDR, &raw, 1, pdMS_TO_TICKS(100));
            if (r != ESP_OK && retry < 2) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        if (r == ESP_OK) {
            x0 = (~raw) & 0xFF;
            app.writeInt16(EzApp::X, 0, static_cast<int16_t>(x0));
            // record DI (inputs) to D offset 100
            app.writeInt16(EzApp::D, 100 * 2, static_cast<int16_t>(x0));
        } else {
            ESP_LOGW(TAG, "Failed read PCF_INPUT after retries: %d", r);
        }
        int64_t t_after_B = esp_timer_get_time();
        uint32_t durB = (uint32_t)(t_after_B - t_after_A);
        secB_sum_us += durB; if (durB < secB_min_us) secB_min_us = durB; if (durB > secB_max_us) secB_max_us = durB; secB_count++;
        // Log X0 bit states
        // {
        //     ESP_LOGI(TAG, "X0: 0x%02X (bits: %d %d %d %d %d %d %d %d)",
        //              x0,
        //              (x0 >> 7) & 1, (x0 >> 6) & 1, (x0 >> 5) & 1, (x0 >> 4) & 1,
        //              (x0 >> 3) & 1, (x0 >> 2) & 1, (x0 >> 1) & 1, (x0 >> 0) & 1);
        // }
        // Check D102; if 0, control Y0 bits 3-4 based on D132 bits 2-3
        int16_t d102 = 0;
        app.readInt16(EzApp::D, curing_step_addr, d102);
        int16_t d131 = 0;
        app.readInt16(EzApp::D, 131 * 2, d131);
        int16_t d132 = 0;
        app.readInt16(EzApp::D, 132 * 2, d132);
        int16_t y0 = 0;
        app.readInt16(EzApp::Y, 0, y0);

        int64_t t_before_C = esp_timer_get_time();
        if (d102 == 0) {
            y0 = ez_dio_control_idle_state(app,d131, d132, x0, y0);
        } else {
            y0 = ez_dio_control_startup_state(app, d102,d131,x0, y0);
        }
        int64_t t_after_C = esp_timer_get_time();
        uint32_t durC = (uint32_t)(t_after_C - t_before_C);
        secC_sum_us += durC; if (durC < secC_min_us) secC_min_us = durC; if (durC > secC_max_us) secC_max_us = durC; secC_count++;

        // 3) Read Y group offset 0 and write to PCF8574 outputs (with retry)
        // write updated Y back so other tasks see the forced state
        app.writeInt16(EzApp::Y, 0, y0);
        int64_t t_before_D = esp_timer_get_time();
        uint8_t out_byte = static_cast<uint8_t>(y0 & 0xFF);
        r = ESP_FAIL;
        for (int retry = 0; retry < 3 && r != ESP_OK; retry++) {
            r = i2c_master_write_to_device(static_cast<i2c_port_t>(EzApp::I2C_PORT), EzApp::PCF_OUTPUT_ADDR, &out_byte, 1, pdMS_TO_TICKS(100));
            if (r != ESP_OK && retry < 2) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "Failed write PCF_OUTPUT after retries: %d", r);
        }
        // record DO (outputs) to D offset 101
        app.writeInt16(EzApp::D, 101 * 2, static_cast<int16_t>(out_byte));
        int64_t t_after_D = esp_timer_get_time();
        uint32_t durD = (uint32_t)(t_after_D - t_before_D);
        secD_sum_us += durD; if (durD < secD_min_us) secD_min_us = durD; if (durD > secD_max_us) secD_max_us = durD; secD_count++;

        // 4) Oxygen readings handled by `oxygen_task` (separate task)

        loop_cnt++;
        int64_t loop_end_us = esp_timer_get_time();
        uint32_t loop_dur_us = (uint32_t)(loop_end_us - loop_start_us);
        // update stats
        loop_sum_us += loop_dur_us;
        if (loop_dur_us < loop_min_us) loop_min_us = loop_dur_us;
        if (loop_dur_us > loop_max_us) loop_max_us = loop_dur_us;
        loop_count++;
        // report every 10 seconds
        if (last_report_time_us == 0) last_report_time_us = loop_end_us;
        if ((loop_end_us - last_report_time_us) >= 10000000LL) {
            double avg_us = loop_count ? ((double)loop_sum_us / (double)loop_count) : 0.0;
            //ESP_LOGI(TAG, "Loop time (us) — min=%u max=%u avg=%.1f count=%u", loop_min_us, loop_max_us, avg_us, loop_count);
            // per-section logs
            double avgA = secA_count ? ((double)secA_sum_us / (double)secA_count) : 0.0;
            double avgB = secB_count ? ((double)secB_sum_us / (double)secB_count) : 0.0;
            double avgC = secC_count ? ((double)secC_sum_us / (double)secC_count) : 0.0;
            double avgD = secD_count ? ((double)secD_sum_us / (double)secD_count) : 0.0;
            // ESP_LOGI(TAG, "Sections A..B (us): A min=%u max=%u avg=%.1f cnt=%u | B min=%u max=%u avg=%.1f cnt=%u",
            //          secA_min_us, secA_max_us, avgA, secA_count,
            //          secB_min_us, secB_max_us, avgB, secB_count);
            // ESP_LOGI(TAG, "Sections C..D (us): C min=%u max=%u avg=%.1f cnt=%u | D min=%u max=%u avg=%.1f cnt=%u",
            //          secC_min_us, secC_max_us, avgC, secC_count,
            //          secD_min_us, secD_max_us, avgD, secD_count);
            // reset loop stats
            loop_sum_us = 0;
            loop_min_us = UINT32_MAX;
            loop_max_us = 0;
            loop_count = 0;
            last_report_time_us = loop_end_us;
            // reset per-section stats
            secA_sum_us = 0; secA_min_us = UINT32_MAX; secA_max_us = 0; secA_count = 0;
            secB_sum_us = 0; secB_min_us = UINT32_MAX; secB_max_us = 0; secB_count = 0;
            secC_sum_us = 0; secC_min_us = UINT32_MAX; secC_max_us = 0; secC_count = 0;
            secD_sum_us = 0; secD_min_us = UINT32_MAX; secD_max_us = 0; secD_count = 0;
            // E (oxygen) handled in oxygen_task and logged separately by sensor code
        }
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
