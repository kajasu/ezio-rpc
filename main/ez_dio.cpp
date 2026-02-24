// ez_dio.cpp
// 디지털 입출력(PCF8574 확장 I/O), I2C 센서(SCD30, E-Oxygen) 관리 모듈
// ADC 다채널 입력, 치료 단계 시퀀스 제어, 음성 출력 번호 관리 포함

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
#include <stdio.h>
#include <time.h>






// small helper to format current local time into a buffer
static inline void __log_get_timestamp(char *buf, size_t len)
{
    time_t __now = time(NULL);
    struct tm __tm;
    localtime_r(&__now, &__tm);
    snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
             __tm.tm_year + 1900, __tm.tm_mon + 1, __tm.tm_mday,
             __tm.tm_hour, __tm.tm_min, __tm.tm_sec);
}

// Logging macro that prefixes messages with current time
// Use esp_log_write() inside the macro to avoid recursive expansion
#define ESP_LOGI_TS(tag, fmt, ...) \
    do { \
        char __log_ts_buf[32]; \
        __log_get_timestamp(__log_ts_buf, sizeof(__log_ts_buf)); \
        esp_log_write(ESP_LOG_INFO, tag, "%s " fmt "\n", __log_ts_buf, ##__VA_ARGS__); \
    } while (0)

#undef ESP_LOGI
#define ESP_LOGI(tag, fmt, ...) ESP_LOGI_TS(tag, fmt, ##__VA_ARGS__)

static const char *TAG = "DIO"; // ESP-IDF 로그 태그

#define EZ_TASK_STACK 4096 // DIO 태스크 스택 크기 (바이트)

// 음성 출력 번호 오프셋 (D 레지스터 바이트 오프셋)
static constexpr int voice_out_no_addr = 109 * 2;
// 치료 단계 D 레지스터 주소 (바이트 오프셋)
static constexpr int curing_step_addr = 102 * 2;


// I2C 버스 스캐너: 가능한 7비트 주소(0x03~0x77)를 스캔하여 장치가 있으면 로그로 보고
static void i2c_scan_bus(i2c_port_t port, const char *port_name)
{
    ESP_LOGI_TS(TAG, "Scanning I2C bus on %s...", port_name);
    int devices_found = 0;
    
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // I2C 명령 링크 생성
        i2c_master_start(cmd);                        // START 조건 전송
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true); // 주소 + 쓰기 비트 (ACK 확인)
        i2c_master_stop(cmd);                         // STOP 조건 전송
        
        esp_err_t err = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50)); // 명령 실행 (50ms 타임아웃)
        i2c_cmd_link_delete(cmd); // 명령 링크 해제
        
        if (err == ESP_OK) {
            ESP_LOGI_TS(TAG, "  Found device at 0x%02X", addr);
            devices_found++;
        }
    }
    
    if (devices_found == 0) {
        ESP_LOGW(TAG, "  No I2C devices found on %s", port_name);
    } else {
        ESP_LOGI_TS(TAG, "  Total %d device(s) found on %s", devices_found, port_name);
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
    conf.master.clk_speed = 10000; // 10kHz (안정성을 위해 낮은 속도 사용)
    conf.clk_flags = 0; // 기본 클럭 소스 사용
    
    esp_err_t i2c_err = i2c_param_config(SENSOR_I2C_PORT, &conf);
    if (i2c_err == ESP_OK) {
        i2c_err = i2c_driver_install(SENSOR_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    }
    
    if (i2c_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize I2C for sensors: %d", i2c_err);
        vTaskDelete(NULL);
        return;
    }
    
    // 클럭 스트레칭(Clock Stretching) 지원을 위한 I2C 타임아웃 설정
    i2c_set_timeout(SENSOR_I2C_PORT, 0xFFFFF); // 클럭 스트레칭을 위한 최대 타임아웃 값
    
    ESP_LOGI_TS(TAG, "Sensor I2C initialized on port %d (SDA=%d, SCL=%d)", 
             SENSOR_I2C_PORT, SENSOR_SDA_IO, SENSOR_SCL_IO);
    
    // I2C 버스 스캔으로 연결된 장치 확인
    i2c_scan_bus(SENSOR_I2C_PORT, "I2C_NUM_1 (Sensors)");
    
    // SCD30 CO2 센서 초기화
    SCD30 airSensor;
    bool scd30_available = false;
    esp_err_t err = airSensor.begin(SENSOR_I2C_PORT, false, true);
    if (err == ESP_OK) {
        ESP_LOGI_TS(TAG, "SCD30 sensor initialized successfully");
        scd30_available = true;
        app.setCO2Status(EzApp::DEVICE_INIT_DONE);
        app.setCO2ErrorCount(0);
    } else {
        ESP_LOGW(TAG, "SCD30 sensor not detected");
        app.setCO2Status(EzApp::DEVICE_ERROR);
        app.incrementCO2ErrorCount();
    }
    
    // DFRobot E-Oxygen 센서 초기화 (주소 0x70)
    DFRobot_EOxygenSensor_I2C oxygenSensor(SENSOR_I2C_PORT, E_OXYGEN_ADDRESS_3);
    bool oxygen_available = false;
    err = oxygenSensor.begin();
    if (err == ESP_OK) {
        ESP_LOGI_TS(TAG, "E-Oxygen sensor initialized successfully");
        oxygen_available = true;
        app.setO2Status(EzApp::DEVICE_INIT_DONE);
        app.setO2ErrorCount(0);
    } else {
        ESP_LOGW(TAG, "E-Oxygen sensor not detected");
        app.setO2Status(EzApp::DEVICE_ERROR);
        app.incrementO2ErrorCount();
    }
    
    // 센서 안정화 대기
    ESP_LOGI_TS(TAG, "센서 안정화 대기...");
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    int wait_count = 0;   // SCD30 데이터 대기 카운터
    int read_cycle = 0;   // 읽기 주기 분산을 위한 카운터
    
    while (1) {
        // SCD30는 기본적으로 2초마다 데이터가 준비되므로 주기적으로 읽음
            if (scd30_available && airSensor.dataAvailable()) {
            uint16_t co2_ppm = airSensor.getCO2();
            float temp_c = airSensor.getTemperature();
            float humidity_pct = airSensor.getHumidity();
            (void)temp_c;
            
            //ESP_LOGI(TAG, "SCD30 - co2: %u ppm, temp: %.1f°C, humidity: %.1f%%", co2_ppm, temp_c, humidity_pct);
            
            // D 레지스터에 측정값 기록:
            // D113: CO2 농도 (ppm)
            // D115: 온도 (x10 스케일, 예: 25.3°C → 253) ← 현재 주석 처리됨
            // D112: 습도 (x100 스케일, 예: 45.2% → 4520)
            app.writeInt16(EzApp::D, 113 * 2, static_cast<int16_t>(co2_ppm));
            //app.writeInt16(EzApp::D, 115 * 2, static_cast<int16_t>(temp_c * 10.0f));
            app.writeInt16(EzApp::D, 112 * 2, static_cast<int16_t>(humidity_pct * 100.0f));
            app.setCO2Status(EzApp::DEVICE_OK);
            wait_count = 0;
        } else if (scd30_available) {
            if (++wait_count % 5 == 1) {
                ESP_LOGI(TAG, "Waiting for new data from SCD30...");
            }
        }
        
        // E-Oxygen 센서 읽기 (I2C 충돌을 피하기 위해 타이밍을 분산시켜 호출)
        if (oxygen_available) {
            float o2_percent = oxygenSensor.readOxygenConcentration();
            
            // 산소 농도를 int16으로 변환 (0.01% 정밀도를 위해 100 배율 적용)
            // 예: 20.95% → 2095
            int16_t o2_val = static_cast<int16_t>(o2_percent * 100.0f);
            
            //ESP_LOGI(TAG, "E-Oxygen - O2: %.2f%% (D117=%d)", o2_percent, o2_val);
            
            // D114 레지스터에 산소 농도 기록
            app.writeInt16(EzApp::D, 114 * 2, o2_val);
            app.setO2Status(EzApp::DEVICE_OK);
        }
        
        read_cycle++;
        
        // SCD30는 기본 측정 주기가 2초이므로 동일 간격으로 대기
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    
    vTaskDelete(NULL);
}

// 제어 로직: 가독성과 테스트 용이성을 위해 메인 루프에서 분리됨
static int16_t ez_dio_control_manual(EzApp &app, int16_t d131 , int16_t d132, int16_t x0, int16_t y0)
{

    // 문열기/닫기: 우선 음성(voice_out)만 재생하고 3초 후 실제 Y0 출력 적용
    static TickType_t door_deadline = 0;
    static int door_pending = 0; // 0: none, 1: close, 2: open
    static int prev_d132 = 0;

    // 문 닫기 명령 감지 (D132.2 상승 에지)
    if ((d132 & 0x0004) && !(prev_d132 & 0x0004)) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 11); // voice: 닫기
        door_pending = 1;
        int16_t delaytime = 0;
        app.readInt16(EzApp::D, 142*2, delaytime);
        delaytime = (delaytime > 0) ? delaytime : 3000; // D102에 설정된 지연 시간 사용, 없으면 기본 3초
        door_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(delaytime); // D102에 설정된 지연 시간 사용, 없으면 기본 3초
    }

    // 문 열기 명령 감지 (D132.3 상승 에지)
    if ((d132 & 0x0008) && !(prev_d132 & 0x0008)) {
        int16_t cur_pressure = 0;   // D111: 현재 압력
        int16_t pressure_limit = 0; // D144: 문 열기 허용 압력 상한

        app.readInt16(EzApp::D, 111 * 2, cur_pressure);
        app.readInt16(EzApp::D, 144 * 2, pressure_limit);

        // 현재 압력이 기준보다 높으면 문 열기 차단
        if (cur_pressure > pressure_limit) {
            app.writeInt16(EzApp::D, voice_out_no_addr, 20); // 압력이 높습니다 안내
            door_pending = 0;
            // door_pending 설정하지 않음 = 문 열기 막음
        } else {
            app.writeInt16(EzApp::D, voice_out_no_addr, 10); // voice: 열기
            door_pending = 2;
            int16_t delaytime = 0;
            app.readInt16(EzApp::D, 142 * 2, delaytime);

            delaytime = (delaytime > 0) ? delaytime : 3000;
            door_deadline = xTaskGetTickCount() + pdMS_TO_TICKS(delaytime);
        }
    }

    // 대기 중인 문 동작의 마감 시간이 지나면 Y0 출력에 실제 변경 적용
    if (door_pending == 1 && (xTaskGetTickCount() >= door_deadline)) {
        // 문 닫기: Y0.0 = 1 (닫기 솔레노이드 ON), Y0.1 = 0 (열기 솔레노이드 OFF)
        y0 |= 0x0001;
        y0 &= ~0x0002;
        door_pending = 0;
    } else if (door_pending == 2 && (xTaskGetTickCount() >= door_deadline)) {
        // 문 열기: Y0.0 = 0 (닫기 솔레노이드 OFF), Y0.1 = 1 (열기 솔레노이드 ON)
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

static int16_t ez_dio_control_auto(EzApp &app, int16_t d102,int16_t d131 ,int16_t x0, int16_t y0)
{

    // Step(=D102) 변화 감지용: 특정 스텝 진입 시 내부 상태(디바운스 등) 초기화에 사용
    static int s_last_step = -1;
    const bool step36_entry = (d102 == 36 && s_last_step != 36);

    // 치료 시작 단계별 시퀀스 타이밍 변수 (단계별 경과 시간 측정)
    static uint32_t start_time = 0;
    static uint32_t start_time_20 = 0;
    static uint32_t start_time_30 = 0;
    static uint32_t start_time_40 = 0;
    static uint32_t start_time_50 = 0;
    static uint32_t start_time_80 = 0;
    static uint32_t stop_time_90 = 0;
    static uint32_t stop_time_92 = 0;
    static uint32_t stop_time_94 = 0;


    (void)time(nullptr);
    
    // 치료종료(3단계 시퀀스): 90 -> 92 -> 94 -> 0
    if (((d131 & 0x0002) || (x0 & 0x0002)|| (x0 & 0x0002)) && (d102 < 90)) { // D131.1 == 1
        app.writeInt16(EzApp::D, curing_step_addr, 90);
        app.writeInt16(EzApp::D, voice_out_no_addr, 8); // 치료가 중지합니다.
        ESP_LOGI(TAG, "[AUTO] 치료 중지 신호 감지, 종료 3단계 시작");

        start_time = 0;
        start_time_20 = 0;
        start_time_30 = 0;
        start_time_40 = 0;
        start_time_50 = 0;
        start_time_80 = 0;
        stop_time_90 = 0;
        stop_time_92 = 0;
        stop_time_94 = 0;
        d102 = 90;
    }
    
    // 종료 1단계: 치료 관련 출력 OFF
    if (d102 == 90) {
        if (stop_time_90 == 0) {
            stop_time_90 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Stop 90] 치료 출력 정지");
        }
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0
        if ((xTaskGetTickCount() - stop_time_90) >= pdMS_TO_TICKS(500)) {
            app.writeInt16(EzApp::D, curing_step_addr, 92);
            ESP_LOGI(TAG, "[Stop 90→92] 출력 정지 완료");
            stop_time_90 = 0;
        }
    // 종료 2단계: 문 열기
    } else if (d102 == 92) {
        if (stop_time_92 == 0) {
            stop_time_92 = xTaskGetTickCount();
            app.writeInt16(EzApp::D, voice_out_no_addr, 10); // 잠시후 문이 열립니다
            ESP_LOGI(TAG, "[Stop 92] 문 열기 준비");
        }
        if ((xTaskGetTickCount() - stop_time_92) >= pdMS_TO_TICKS(4000)) {
                int16_t cur_pressure = 0;   // D111: 현재 압력
                int16_t pressure_limit = 0; // D144: 문 열기 허용 압력 상한

                app.readInt16(EzApp::D, 111 * 2, cur_pressure);
                app.readInt16(EzApp::D, 144 * 2, pressure_limit);

                // 현재 압력이 기준보다 높으면 문 열기 차단
                if (cur_pressure > pressure_limit) {
                    app.writeInt16(EzApp::D, voice_out_no_addr, 20); // 압력이 높습니다 안내
                    ESP_LOGW(TAG, "[Stop 92] 문 열기 차단 - 현재 압력 %d > 허용 압력 %d", cur_pressure, pressure_limit);
                    app.writeInt16(EzApp::D, curing_step_addr, 94);
                    ESP_LOGI(TAG, "[Stop 92→94] 문 열기 출력 적용");
                    stop_time_92 = 0;
                } else {
                    ESP_LOGI(TAG, "[Stop 92] 문 열기 허용 - 현재 압력 %d <= 허용 압력 %d", cur_pressure, pressure_limit);
                    y0 |= 0x0002;  // Y0.1 = 1
                    y0 &= ~0x0001; // Y0.0 = 0
                    app.writeInt16(EzApp::D, curing_step_addr, 94);
                    ESP_LOGI(TAG, "[Stop 92→94] 문 열기 출력 적용");
                    stop_time_92 = 0;
                }            
        }
    // 종료 3단계: 종료 완료 및 수동모드 복귀
    } else if (d102 == 94) {
        if (stop_time_94 == 0) {
            stop_time_94 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Stop 94] 종료 마무리");
        }
        if ((xTaskGetTickCount() - stop_time_94) >= pdMS_TO_TICKS(500)) {
            app.writeInt16(EzApp::D, curing_step_addr, 0);
            ESP_LOGI(TAG, "[Stop 94→0] 종료 3단계 완료, 수동모드 복귀");
            stop_time_94 = 0;
        }

    //문닫고 ->  가압 -> 치료 ->  배출 -> 문열기
    } else if (d102 == 10) {
        y0 &= ~0x0001; // Y0.0 = 0
        y0 &= ~0x0002; // Y0.1 = 0
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0
        start_time_20 = 0;
        start_time_30 = 0;
        start_time_40 = 0;
        start_time_50 = 0;
        start_time_80 = 0;
        app.writeInt16(EzApp::D, voice_out_no_addr, 1); //치료를 시작합니다
        if (start_time == 0) {
            start_time = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 10] 치료 시작 대기 중...");
        }
        if ((xTaskGetTickCount() - start_time) >= pdMS_TO_TICKS(3000)) {
            app.writeInt16(EzApp::D, curing_step_addr, 20);
            ESP_LOGI(TAG, "[Step 10→20] 3초 경과, 다음 단계로 전환");
            start_time = 0;
        }
        
    } else if (d102 == 20) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 11); //잠시후 문이 닫힙니다
        if (start_time_20 == 0) {
            start_time_20 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 20] 문 닫기 시작");
        }
        int16_t delaytime = 0;
        app.readInt16(EzApp::D, 142*2, delaytime);
        delaytime = (delaytime > 0) ? delaytime : 3000;
        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(delaytime)) {
            y0 |= 0x0001;  // Y0.0 = 1
            y0 &= ~0x0002; // Y0.1 = 0
            app.writeInt16(EzApp::D, curing_step_addr, 22);
            ESP_LOGI(TAG, "[Step 20→22] 200ms 경과, 문 닫힘 대기 단계로 전환");
            start_time_20 = 0;
        }
        } else if (d102 == 22) {
        if (start_time_20 == 0) {
            start_time_20 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 22] 문 닫힘 대기 중");
        }

        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(1000)) {
            app.writeInt16(EzApp::D, curing_step_addr, 24);
            ESP_LOGI(TAG, "[Step 22→24] %u ms 경과, 다음 단계로 전환", 1000U);
            start_time_20 = 0;
        }
        } else if (d102 == 24) {
        if (start_time_20 == 0) {
            start_time_20 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 24] 문 닫힘 확인");
        }
        if ((xTaskGetTickCount() - start_time_20) >= pdMS_TO_TICKS(500)) {
            app.writeInt16(EzApp::D, curing_step_addr, 30);
            ESP_LOGI(TAG, "[Step 24→30] 500ms 경과, 가압 단계로 전환");
            start_time_20 = 0;
        }
    } else if (d102 == 30) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 12); //압력 제어를 시작합니다.
        if (start_time_30 == 0) {
            start_time_30 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 30] 가압 단계 시작");
        }
        if ((xTaskGetTickCount() - start_time_30) >= pdMS_TO_TICKS(2500)) {
            y0 |= 0x0004;  // Y0.2 = 1
            app.writeInt16(EzApp::D, curing_step_addr, 32);
            ESP_LOGI(TAG, "[Step 30→32] 8초 경과, 온도 제어 단계로 전환");
            start_time_30 = 0;
        }
    } else if (d102 == 32) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 16); //온도 제어를 시작합니다.
        if (start_time_30 == 0) {
            start_time_30 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 32] 온도 제어 단계 시작");
        }
        
        int16_t d149 = 0;
        app.readInt16(EzApp::D, 149 * 2, d149);
        
        if (d149 == 1) {
            if ((xTaskGetTickCount() - start_time_30) >= pdMS_TO_TICKS(2500)) {
            y0 |= 0x0010;  // Y0.4 = 1
            app.writeInt16(EzApp::D, curing_step_addr, 34);
            ESP_LOGI(TAG, "[Step 32→34] 온도 제어 완료, 다음 단계로 전환");
            start_time_30 = 0;
            }
        } else {
            app.writeInt16(EzApp::D, curing_step_addr, 34);
            ESP_LOGI(TAG, "[Step 32→34] D149 != 1, 온도 제어 스킵");
            start_time_30 = 0;
        }
    } else if (d102 == 34) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 18); //산소공급을 시작합니다
        if (start_time_30 == 0) {
            start_time_30 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 34] 산소 시작");
        }
        if ((xTaskGetTickCount() - start_time_30) >= pdMS_TO_TICKS(2500)) {
            y0 |= 0x0020;  // Y0.5 = 1   
            app.writeInt16(EzApp::D, curing_step_addr, 36);
            ESP_LOGI(TAG, "[Step 34→36] 2초 경과, 다음 단계로 전환");
            start_time_30 = 0;
        }
    } else if (d102 == 36) {
        app.writeInt16(EzApp::D, voice_out_no_addr, 21); //마무리 단계입니다.
        

        int16_t cur_pressure = 0;   // D111: 현재 압력
        int16_t pressure_sv = 0; // D144: 설정 압력
        int16_t pressure_margin = 0; // D145: 압력 마진

        app.readInt16(EzApp::D, 111 * 2, cur_pressure);
        app.readInt16(EzApp::D, 141 * 2, pressure_sv);
        app.readInt16(EzApp::D, 145 * 2, pressure_margin);

        int16_t duration_min = 0;
        app.readInt16(EzApp::D, 143 * 2, duration_min);
        if (duration_min <= 0) {
            duration_min = 20; // 기본 20분  
            app.writeInt16(EzApp::D, 143 * 2, duration_min);
        }
        const TickType_t wait_ticks = pdMS_TO_TICKS(static_cast<uint32_t>(duration_min) * 60U * 1000U);

        if (start_time_30 == 0) {
            start_time_30 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 36] 치료단계 시작 - 치료시간: %d분", duration_min);
     
            ESP_LOGI(TAG, "[Step 36] 현재 압력: %d, 설정 압력: %d, 압력 마진: %d", 
                 cur_pressure, pressure_sv, pressure_margin);

        }


        
        const int16_t pressure_threshold = static_cast<int16_t>(pressure_sv + pressure_margin);

        // 출력 디바운스(안정화): 압력 조건이 변해도 즉시 반응하지 않고,
        // 조건이 3초 이상 연속 유지될 때만 Y0 비트를 변경한다.
        static int s_press_target = -1; // 0: off, 1: on, -1: uninitialized
        static TickType_t s_press_since = 0;
        static int s_vent_target = -1;  // 0: off, 1: on, -1: uninitialized
        static TickType_t s_vent_since = 0;

        const TickType_t now = xTaskGetTickCount();
        const TickType_t stable_ticks = pdMS_TO_TICKS(3000);

        // Step36에 막 진입한 경우: 이전 스텝에서 남아있는 타이머/목표값을 초기화
        if (step36_entry) {
            s_press_target = -1;
            s_press_since = now;
            s_vent_target = -1;
            s_vent_since = now;
        }

        const int desired_press = (cur_pressure > pressure_sv) ? 0 : 1; // Y0.2
        const int desired_vent = (cur_pressure > pressure_threshold) ? 1 : 0; // Y0.3

        if (s_press_target != desired_press) {
            s_press_target = desired_press;
            s_press_since = now;
        }
        if (s_vent_target != desired_vent) {
            s_vent_target = desired_vent;
            s_vent_since = now;
        }

        const int cur_press_bit = (y0 & 0x0004) ? 1 : 0;
        const int cur_vent_bit = (y0 & 0x0008) ? 1 : 0;

        // 첫 계산에서는 현재 출력 상태를 기준으로 초기화하여, 불필요한 즉시 토글을 방지
        if (s_press_target < 0) {
            s_press_target = cur_press_bit;
            s_press_since = now;
        }
        if (s_vent_target < 0) {
            s_vent_target = cur_vent_bit;
            s_vent_since = now;
        }

        if (cur_press_bit != s_press_target && (now - s_press_since) >= stable_ticks) {
            if (s_press_target) y0 |= 0x0004;  // Y0.2 = 1 (가압)
            else y0 &= ~0x0004;                // Y0.2 = 0 (가압 중지)
        }

        if (cur_vent_bit != s_vent_target && (now - s_vent_since) >= stable_ticks) {
            if (s_vent_target) y0 |= 0x0008;   // Y0.3 = 1 (압력벤트)
            else y0 &= ~0x0008;                // Y0.3 = 0 (압력벤트 중지)
        }

        //압력이 초과하면 배출, 가압중지 , 압력이 기준이하면 배출금지 , 가압   

        // 경과시간(분) 계산해서 D119에 저장
        uint32_t elapsed_ms = xTaskGetTickCount() - start_time_30;
        int16_t elapsed_min = static_cast<int16_t>(elapsed_ms / (60U * 1000U));
        app.writeInt16(EzApp::D, 119 * 2, elapsed_min);
        
        if (elapsed_ms >= wait_ticks) {
            y0 &= ~0x0004;  // Y0.2 = 0
            y0 &= ~0x0010;  // Y0.4 = 0
            y0 |= 0x0008;  // Y0.3 = 1 (압력벤트 시작)
            app.writeInt16(EzApp::D, curing_step_addr, 40);
            ESP_LOGI(TAG, "[Step 36→40] 치료시간 대기 (%d분)", duration_min);
            start_time_30 = 0;
        }
    } else if (d102 == 40) {
        if (start_time_40 == 0) {
            start_time_40 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 40] 배출 단계 시작");
        }
        if ((xTaskGetTickCount() - start_time_40) >= pdMS_TO_TICKS(2000)) {         
            app.writeInt16(EzApp::D, curing_step_addr, 50);
            ESP_LOGI(TAG, "[Step 40→50] 2초 경과, 배출 단계로 전환");
            start_time_40 = 0;
        }
    } else if (d102 == 50) {
        int16_t cur_pressure = 0;   // D111: 현재 압력
        int16_t pressure_limit = 0; // D144: 배출 완료 기준 압력
        app.writeInt16(EzApp::D, voice_out_no_addr, 14); //배출을 시작합니다.
        if (start_time_50 == 0) {
            start_time_50 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 50] 배출 단계 시작");
        }
        
        app.readInt16(EzApp::D, 111 * 2, cur_pressure);
        app.readInt16(EzApp::D, 144 * 2, pressure_limit);

        // 현재 압력이 기준보다 높으면 문 열기 차단
        if (cur_pressure < pressure_limit) {
            y0 |= 0x0008;  // Y0.3 = 1
            app.writeInt16(EzApp::D, curing_step_addr, 80);
            ESP_LOGI(TAG, "[Step 50→80] 배출 완료, 압력 정상, 문 열기 단계로 전환");
            start_time_50 = 0;
        }
    } else if (d102 == 80) {
        if (start_time_80 == 0) {
            start_time_80 = xTaskGetTickCount();
            ESP_LOGI(TAG, "[Step 80] 문 열기 단계 시작");
        }   
        app.writeInt16(EzApp::D, voice_out_no_addr, 10); //잠시후 문이 열립니다
        if ((xTaskGetTickCount() - start_time_80) >= pdMS_TO_TICKS(3000)) {
            y0 |= 0x0002;  // Y0.1 = 1
            y0 &= ~0x0001; // Y0.0 = 0
            app.writeInt16(EzApp::D, curing_step_addr, 99);
            ESP_LOGI(TAG, "[Step 80→99] 3초 경과, 문 열기 명령 발령");
            start_time_80 = 0;
        }
    } else if (d102 == 99) {
        if (start_time_80 == 0) {  // 첫 진입 시만 로그
            ESP_LOGI(TAG, "[Step 99] 치료 완료, 모든 출력 OFF");
            start_time_80 = 1;  // 로그 중복 방지
        }
        y0 &= ~0x0004; // Y0.2 = 0
        y0 &= ~0x0008; // Y0.3 = 0
        y0 &= ~0x0010; // Y0.4 = 0
        y0 &= ~0x0020; // Y0.5 = 0
        app.writeInt16(EzApp::D, curing_step_addr, 0);
        app.writeInt16(EzApp::D, voice_out_no_addr, 8); //치료가 중지합니다.
    }

    s_last_step = d102;
    return y0;
}

// 산소 센서 별도 태스크 삭제됨 (기능이 oxygen_sensor_task로 통합됨)

// 메인 DIO 제어 태스크: PCF8574 입출력 처리, 제어 상태 머신 실행
static void ez_dio_task(void *arg)
{

    (void)arg;
    EzApp &app = EzApp::instance();
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
        // 1) 루프 카운터를 D 그룹 오프셋 0에 기록 (int32, 워치독/디버깅용)
        app.writeInt32(EzApp::D, 0, loop_cnt++);

        // 2) PCF8574에서 디지털 입력 읽기, X 그룹 오프셋 0에 저장 (최대 3회 재시도)
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
            // 디지털 입력(DI) 값을 D 오프셋 100(D100)에 기록 (HMI 모니터링용)
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
            y0 = ez_dio_control_manual(app,d131, d132, x0, y0);
        } else {
            y0 = ez_dio_control_auto(app, d102,d131,x0, y0);
        }


        //온도제어(자동여보와 관계없이)
        if (d131 & 0x0004) { // D131.2 == 1
            app.writeInt16(EzApp::D, voice_out_no_addr, 16);
            y0 |= 0x0010;  // Y0.0 = 1
        } else if (d131 & 0x0008) { // D131.3 == 1
            app.writeInt16(EzApp::D, voice_out_no_addr, 17);
            y0 &= ~0x0010; // Y0.0 = 0
        }

        {
            static int16_t last_voice_val = 0;
            static TickType_t stable_since = 0;

            int16_t voice_val = 0;
            app.readInt16(EzApp::D, voice_out_no_addr, voice_val);
            TickType_t now = xTaskGetTickCount();

            if (voice_val != last_voice_val) {
                last_voice_val = voice_val;
                stable_since = now;
            } else if (voice_val != 0 && (now - stable_since) >= pdMS_TO_TICKS(3000)) {
                app.writeInt16(EzApp::D, voice_out_no_addr, 0);
                last_voice_val = 0;
                stable_since = now;
            }
        }

        // 3) 변경된 Y 값을 공유 메모리에 기록하고 PCF8574로 실제 디지털 출력 (최대 3회 재시도)
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
        // 디지털 출력(DO) 값을 D 오프셋 101(D101)에 기록 (HMI 모니터링용)
        app.writeInt16(EzApp::D, 101 * 2, static_cast<int16_t>(out_byte));

        // 4) 산소 센서 읽기는 별도 태스크(oxygen_sensor_task)에서 처리됨

        loop_cnt++;
        // I2C/ADC 동작을 위한 RTOS 스케줄링 최소 대기
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    vTaskDelete(NULL);
}

// DIO 태스크 시작: PCF8574 디지털 I/O 처리 및 치료 단계 제어 로직 수행
void start_ez_dio_task()
{
    xTaskCreate(ez_dio_task, "ez_dio_task", EZ_TASK_STACK, NULL, 5, NULL);
}

// Y0 토글 테스트 태스크: Y 오프셋 0의 비트 0~7을 2초 간격으로 순환 출력 (디버깅용)
static void y0_toggle_task(void *arg)
{
    (void)arg;
    // 현재 테스트 비활성화 (본문 주석 처리됨)
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

// Y0 토글 테스트 태스크 시작 (디버깅 목적)
void start_y0_toggle_test()
{
    xTaskCreate(y0_toggle_task, "y0_toggle", 2048, NULL, 5, NULL);
}
