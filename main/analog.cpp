#include "analog.h"
#include "ezapp.h"
#include "adc_mgr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/dac_oneshot.h" // dac_oneshot 신규 API
#include "esp_log.h"

#define ANALOG_TASK_STACK 4096

// GPIO mapping (per user)
// DA1 -> GPIO26 (DAC channel 2)
// DA2 -> GPIO25 (DAC channel 1)
// A1 -> GPIO36 (ADC1_CH0)
// A2 -> GPIO39 (ADC1_CH3)
// A3 -> GPIO34 (ADC1_CH6)
// A4 -> GPIO35 (ADC1_CH7)

static void analog_task(void *arg)
{
    (void)arg;
    EzApp &app = EzApp::instance();

    // Initialize shared ADC manager once (safe to call repeatedly)
    esp_err_t aerr = adc_mgr_init();
    if (aerr != ESP_OK) {
        ESP_LOGE("analog", "adc_mgr_init failed: %d", aerr);
    }

    // DAC 주의: DA1=GPIO26(DAC_CHAN_1), DA2=GPIO25(DAC_CHAN_0)
    // dac_oneshot 신규 API로 초기화
    dac_oneshot_handle_t dac_chan0 = nullptr; // GPIO25 (DA2)
    dac_oneshot_handle_t dac_chan1 = nullptr; // GPIO26 (DA1)
    dac_oneshot_config_t cfg0 = { .chan_id = DAC_CHAN_0 };
    dac_oneshot_config_t cfg1 = { .chan_id = DAC_CHAN_1 };
    dac_oneshot_new_channel(&cfg0, &dac_chan0);
    dac_oneshot_new_channel(&cfg1, &dac_chan1);

    // D111용 이동평균 버퍼 (윈도우 크기)
    // 설명:
    //  - 원시 ADC 값(raw1)을 -200..1000 범위로 매핑한 후 이동평균을 취합니다.
    //  - 순환 버퍼 방식으로 구현하여 메모리/연산 비용을 절감합니다.
    //  - 초기에는 버퍼가 완전히 채워질 때까지 평균 갯수(ma_count)가 증가합니다.
    static const int MA_WINDOW = 100; // 이동평균 윈도우 크기 (변경 가능)
    static int32_t ma_buf[MA_WINDOW] = {0}; // 윈도우 값 저장소
    static int ma_idx = 0;                 // 순환 인덱스
    static int ma_count = 0;               // 현재 버퍼에 채워진 개수 (최대 MA_WINDOW)
    static int32_t ma_sum = 0;             // 현재 합계 (평균 계산을 위해 유지)
    int ch1_zero = 634; // ADC raw value corresponding to 0 in the -200..1000 scale (calibrated)
    
    float ch1_gain = 3.05f; // ADC raw value corresponding to 0 in the -200..1000 scale (calibrated)

    while (1) {
        // Read A1 (ADC1_CH0 -> GPIO36)
        int raw1 = 0;
        int raw2 = 0;
        (void)adc_mgr_read(ADC_CHANNEL_0, &raw1);
        (void)adc_mgr_read(ADC_CHANNEL_3, &raw2);
        
        // Allow other tasks to run
        taskYIELD();

        // store to D offsets 522 and 524 as int16 (clamp to 0..4095)
        //ESP_LOGI("analog", "raw1=%d, raw2=%d", raw1, raw2);
        if (raw1 < 0) raw1 = 0;
        if (raw1 > 4095) raw1 = 4095;
        if (raw2 < 0) raw2 = 0;
        if (raw2 > 4095) raw2 = 4095;
        // EzApp offsets are BYTES, so D112 => 112*2
        //app.writeInt16(EzApp::D, 112 * 2, static_cast<int16_t>(raw1));
        // 1) 선형 매핑: ADC(0..4095) -> 범위(-200..1000)
        //    수식 유도:
        //      목표 범위 폭 = 1000 - (-200) = 1200
        //      따라서 out = raw1 * 1200/4095 + (-200)
        //    정수 연산을 사용하므로 오버플로우 주의 (tmp는 32비트 사용)
        // int32_t tmp = static_cast<int32_t>(raw1) * 1200;
        // int d111 = static_cast<int>(tmp / 4095) - 200; // 매핑 결과
        //int d111 = static_cast<int>(0.305f * static_cast<float>(raw1 - 634)); // 매핑 결과
        // 2) 클램핑: 안전을 위해 결과를 지정 범위로 제한
        //if (d111 < -200) d111 = -200;
        //if (d111 > 1000) d111 = 1000;
        // 3) 이동평균 적용 (순환 버퍼)
        //    - ma_buf: 이전 N개 샘플 보관
        //    - ma_sum: 현재 합 (새 값 추가 시 빠르게 평균 계산 가능)
        //    - ma_idx: 순환 인덱스 (덮어쓰기 위치)
        //    - ma_count: 현재 버퍼에 채워진 요소 수 (초기 가변)
        int replaced = ma_buf[ma_idx];      // 덮어써질 이전 값
        ma_sum = ma_sum - replaced + raw1;  // 합 갱신
        ma_buf[ma_idx] = raw1;              // 새 값 저장
        ma_idx = (ma_idx + 1) % MA_WINDOW;  // 인덱스 증분 (순환)
        if (ma_count < MA_WINDOW) ma_count++;
        int raw1_avg = static_cast<int>(ma_sum / (ma_count == 0 ? 1 : ma_count));
        int d111_avg = static_cast<int>(ch1_gain * static_cast<float>(raw1_avg - ch1_zero)); // 매핑 결과
        
        static int log_count = 0;
        if (++log_count >= 200) {
            //ESP_LOGI("analog", "raw1_avg=%d, d111_avg=%d", raw1_avg, d111_avg);
            int16_t cal_zero = 0;
            app.readInt16(EzApp::D, 146 * 2, cal_zero); // D146에서 보정값 읽기 (예: 634)            
            ch1_zero = (cal_zero == 0) ? 634 : cal_zero; // 보정값이 0이면 기본값 사용

            int16_t cal_gain = 0;
            app.readInt16(EzApp::D, 147 * 2, cal_gain); // D147에서 보정값 읽기 (예: 305 -> 0.305)
            ch1_gain = (cal_gain == 0) ? 0.305f : (cal_gain / 1000.0f); // 보정값이 0이면 기본값 사용
            //ESP_LOGI("analog", "Calibration: ch1_zero=%d, ch1_gain=%.3f", ch1_zero, ch1_gain);
            log_count = 0;
        }
        // 4) 결과를 D111에 기록 (정수, 이미 스케일 조정됨)
        app.writeInt16(EzApp::D, 111 * 2, static_cast<int16_t>(d111_avg));
        app.writeInt16(EzApp::D, 116 * 2, static_cast<int16_t>(raw1_avg));

        // D 그룹에서 출력값을 읽어 DAC로 출력
        // (예: D10/D12에 원하는 아날로그 값이 채워져 있으면 DAC로 변환해서 출력)
        int16_t v10 = 0;
        int16_t v12 = 0;
        app.readInt16(EzApp::D,  111 * 2, v10); // 예시: D111을 DA1으로 출력할 수도 있음
        app.readInt16(EzApp::D,  114 * 2, v12);
        
        // Allow other tasks to run before DAC operations
        taskYIELD();
        
        // int16 범위(-32768..32767)을 DAC 출력(0..255)으로 정규화
        // 설명: DAC는 8비트(0..255) 입력을 기대하므로 선형 비례 매핑 수행
        auto map_to_dac = [](int16_t val)->uint8_t {
            int32_t shifted = static_cast<int32_t>(val) + 32768; // 0..65535로 쉬프트
            uint32_t out = (static_cast<uint32_t>(shifted) * 255u) / 65535u; // 정규화
            if (out > 255u) out = 255u;
            return static_cast<uint8_t>(out);
        };

        uint8_t dac_val1 = map_to_dac(v10); // D111 등 원하는 D 레지스터를 DAC로 매핑 가능
        uint8_t dac_val2 = map_to_dac(v12);

        // Write to DAC channels
        // Note: user mapping: DA1=GPIO26 (DAC_CHAN_1), DA2=GPIO25 (DAC_CHAN_0)
        dac_oneshot_output_voltage(dac_chan1, dac_val1);
        dac_oneshot_output_voltage(dac_chan0, dac_val2);
        // Update EzApp current time
        //app.updateCurrentTime();

        // Increased delay to allow other tasks to reset watchdog
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    vTaskDelete(NULL);
}

void start_analog_task()
{
    xTaskCreate(analog_task, "analog_task", ANALOG_TASK_STACK, NULL, 5, NULL);
}
