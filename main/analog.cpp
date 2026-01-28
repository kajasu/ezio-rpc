#include "analog.h"
#include "ezapp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/dac.h"
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

    // configure ADC1 width and attenuations
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_12); // A1 -> GPIO36
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12); // A2 -> GPIO39
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12); // A3 -> GPIO34 (unused here)
    adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_12); // A4 -> GPIO35 (unused here)

    // configure DAC outputs
    dac_output_enable(DAC_CHAN_0); // GPIO25 (DA2)
    dac_output_enable(DAC_CHAN_1); // GPIO26 (DA1)

    while (1) {
        // Read A1 (ADC1_CH0 -> GPIO36)
        int raw1 = adc1_get_raw(ADC1_CHANNEL_0); // 0..4095
        int raw2 = adc1_get_raw(ADC1_CHANNEL_3); // A2

        // store to D offsets 522 and 524 as int16 (clamp to 0..4095)
        if (raw1 < 0) raw1 = 0;
        if (raw1 > 4095) raw1 = 4095;
        if (raw2 < 0) raw2 = 0;
        if (raw2 > 4095) raw2 = 4095;
        app.writeInt16(EzApp::D, 522, static_cast<int16_t>(raw1));
        app.writeInt16(EzApp::D, 524, static_cast<int16_t>(raw2));

        // Read D 10 and D 12 (int16) and output to DAC1/2
        int16_t v10 = 0;
        int16_t v12 = 0;
        app.readInt16(EzApp::D, 10, v10);
        app.readInt16(EzApp::D, 12, v12);

        // Map int16 (-32768..32767) to 0..255 for DAC
        auto map_to_dac = [](int16_t val)->uint8_t {
            int32_t shifted = static_cast<int32_t>(val) + 32768; // 0..65535
            uint32_t out = (static_cast<uint32_t>(shifted) * 255u) / 65535u;
            if (out > 255u) out = 255u;
            return static_cast<uint8_t>(out);
        };

        uint8_t dac_val1 = map_to_dac(v10); // DA1? we'll assign: D10->DA1(GPIO26)
        uint8_t dac_val2 = map_to_dac(v12); // D12->DA2(GPIO25)

        // Write to DAC channels
        // Note: user mapping: DA1=GPIO26 (DAC_CHANNEL_2), DA2=GPIO25 (DAC_CHANNEL_1)
        dac_output_voltage(DAC_CHAN_1, dac_val1);
        dac_output_voltage(DAC_CHAN_0, dac_val2);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelete(NULL);
}

void start_analog_task()
{
    xTaskCreate(analog_task, "analog_task", ANALOG_TASK_STACK, NULL, 5, NULL);
}
