#include "ez_dio.h"
#include "ezapp.h"
#include "adc_mgr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "DFRobot_EOxygenSensor.h"
#include "SparkFun_SCD30_Arduino_Library.h"
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

// 음성 출력 번호 오프셋 (D 레지스터 바이트 오프셋)
static constexpr int voice_out_no_addr = 109 * 2;
// 치료 단계 D 레지스터 주소 (바이트 오프셋)
static constexpr int curing_step_addr = 102 * 2;



static adc_channel_t adc_ch1 = ADC_CHANNEL_0; // ADC 채널 0 -> GPIO36 (ADC1)
static adc_channel_t adc_ch2 = ADC_CHANNEL_3; // ADC 채널 3 -> GPIO39 (ADC1)
static adc_channel_t adc_ch3 = ADC_CHANNEL_6; // ADC 채널 6 -> GPIO34 (ADC1)
static adc_channel_t adc_ch4 = ADC_CHANNEL_7; // ADC 채널 7 -> GPIO35 (ADC1)

// I2C helper functions removed (oxygen sensor code deleted)

// I2C 버스 스캐너: 가능한 7비트 주소(0x03~0x77)를 스캔하여 장치가 있으면 로그로 보고
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

// 다중 센서 태스크: SCD30(CO2/온도/습도) 및 E-Oxygen(산소) 센서를 읽음
static void oxygen_sensor_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();
    
    // 센서용 별도 I2C 포트 설정 (I2C_NUM_1)
    // GPIO 5 = SCL, GPIO 22 = SDA
    const i2c_port_t SENSOR_I2C_PORT = I2C_NUM_1;
    const int SENSOR_SDA_IO = 22;
    const int SENSOR_SCL_IO = 5;
    
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SENSOR_SDA_IO;
    conf.scl_io_num = SENSOR_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 50000; // 50kHz (slower for better reliability)
    conf.clk_flags = 0; // Use default clock source
    
    esp_err_t i2c_err = i2c_param_config(SENSOR_I2C_PORT, &conf);
    if (i2c_err == ESP_OK) {
        i2c_err = i2c_driver_install(SENSOR_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C for sensors: %d", i2c_err);
        vTaskDelete(NULL);
        return;
    }
    
    // Set I2C timeout for clock stretching support
    i2c_set_timeout(SENSOR_I2C_PORT, 0xFFFFF); // Max timeout for clock stretching
    
    ESP_LOGI(TAG, "Sensor I2C initialized on port %d (SDA=%d, SCL=%d)", 
             SENSOR_I2C_PORT, SENSOR_SDA_IO, SENSOR_SCL_IO);
    
    // I2C 버스 스캔으로 연결된 장치 확인
    i2c_scan_bus(SENSOR_I2C_PORT, "I2C_NUM_1 (Sensors)");
    
    // SCD30 CO2 센서 초기화
    SCD30 airSensor;
    bool scd30_available = false;
    esp_err_t err = airSensor.begin(SENSOR_I2C_PORT, false, true);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SCD30 sensor initialized successfully");
        scd30_available = true;
    } else {
        ESP_LOGW(TAG, "SCD30 sensor not detected");
    }
    
    // DFRobot E-Oxygen 센서 초기화 (주소 0x70)
    DFRobot_EOxygenSensor_I2C oxygenSensor(SENSOR_I2C_PORT, E_OXYGEN_ADDRESS_3);
    bool oxygen_available = false;
    err = oxygenSensor.begin();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "E-Oxygen sensor initialized successfully");
        oxygen_available = true;
    } else {
        ESP_LOGW(TAG, "E-Oxygen sensor not detected");
    }
    
    // 센서 안정화 대기
    ESP_LOGI(TAG, "센서 안정화 대기...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    int wait_count = 0;
    int read_cycle = 0; // Counter to stagger reads
    
    while (1) {
        // SCD30는 기본적으로 2초마다 데이터가 준비되므로 주기적으로 읽음
            if (scd30_available && airSensor.dataAvailable()) {
            uint16_t co2_ppm = airSensor.getCO2();
            float temp_c = airSensor.getTemperature();
            float humidity_pct = airSensor.getHumidity();
            
            ESP_LOGI(TAG, "SCD30 - co2: %u ppm, temp: %.1f°C, humidity: %.1f%%", 
                     co2_ppm, temp_c, humidity_pct);
            
            // Write to D registers:
            // D114: CO2 (ppm)
            // D115: Temperature (scaled by 10, e.g., 25.3°C = 253)
            // D116: Humidity (scaled by 10, e.g., 45.2% = 452)
            app.writeInt16(EzApp::D, 113 * 2, static_cast<int16_t>(co2_ppm));
            //app.writeInt16(EzApp::D, 115 * 2, static_cast<int16_t>(temp_c * 10.0f));
            app.writeInt16(EzApp::D, 112 * 2, static_cast<int16_t>(humidity_pct * 100.0f));
            wait_count = 0;
        } else if (scd30_available) {
            if (++wait_count % 5 == 1) {
                ESP_LOGI(TAG, "Waiting for new data from SCD30...");
            }
        }
        
        // E-Oxygen 센서 읽기 (I2C 충돌을 피하기 위해 타이밍을 분산시켜 호출)
        if (oxygen_available) {
            float o2_percent = oxygenSensor.readOxygenConcentration();
            
            // Convert to int16 (scaled by 100 for 0.01% precision)
            // e.g., 20.95% becomes 2095
            int16_t o2_val = static_cast<int16_t>(o2_percent * 100.0f);
            
            ESP_LOGI(TAG, "E-Oxygen - O2: %.2f%% (D117=%d)", o2_percent, o2_val);
            
            // Write to D117 register
            app.writeInt16(EzApp::D, 114 * 2, o2_val);
        }
        
        read_cycle++;
        
        // SCD30 has data ready every 2 seconds by default
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    vTaskDelete(NULL);
}

    // 제어 로직: 가독성과 테스트 용이성을 위해 메인 루프에서 분리됨
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
    // Loop timing statistics removed to reduce log noise and overhead.

    // Oxygen sensor reads are handled in a separate task started below.
    // Initialize shared ADC manager once (safe to call repeatedly).
    esp_err_t adc_init_err = adc_mgr_init();
    if (adc_init_err != ESP_OK) {
        ESP_LOGE(TAG, "adc_mgr_init failed: %d", adc_init_err);
    }
    
    // I2C_NUM_0 버스에서 PCF8574 및 기타 장치 스캔
    static bool i2c_scanned = false;
    if (!i2c_scanned) {
        i2c_scan_bus(static_cast<i2c_port_t>(EzApp::I2C_PORT), "I2C_NUM_0 (PCF8574)");
        i2c_scanned = true;
    }
    
    // 다중 센서 태스크 시작 (SCD30, E-Oxygen)
    static bool sensor_task_started = false;
    if (!sensor_task_started) {
        xTaskCreate(oxygen_sensor_task, "multi_sensor", 4096, NULL, 4, NULL);
        sensor_task_started = true;
        ESP_LOGI(TAG, "다중 센서 태스크 시작 (SCD30 + E-Oxygen)");
    }
    
    int loop_cnt = 0;
    int16_t x0 = 0;
    while (1) {
        // 1) Increment D group at offset 0 (int32)
        app.writeInt32(EzApp::D, 0, loop_cnt++);

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
        int16_t d102 = 0;
        app.readInt16(EzApp::D, curing_step_addr, d102);
        int16_t d131 = 0;
        app.readInt16(EzApp::D, 131 * 2, d131);
        int16_t d132 = 0;
        app.readInt16(EzApp::D, 132 * 2, d132);
        int16_t y0 = 0;
        app.readInt16(EzApp::Y, 0, y0);

        if (d102 == 0) {
            y0 = ez_dio_control_idle_state(app,d131, d132, x0, y0);
        } else {
            y0 = ez_dio_control_startup_state(app, d102,d131,x0, y0);
        }

        // 3) Write updated Y back so other tasks see the forced state and output via PCF8574 (with retry)
        app.writeInt16(EzApp::Y, 0, y0);
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

        // 4) Oxygen readings handled by `oxygen_task` (separate task)

        loop_cnt++;
        // RTOS scheduling small delay to allow I2C/ADC activity
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
