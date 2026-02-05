#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "ezapp.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "ez_dio.h"
#include "modbus.h"
#include "analog.h"
#include <stdlib.h>
#include <string.h>

#define BLINK_GPIO GPIO_NUM_2
/*

DI
    알람
    비상정지
    스타트
    스톱
DO
    칠러런
    에어벤트
    압력
    도어오픈
    도어클로즈
*/
extern "C" void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);

    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // Initialize EzApp singleton and demo typed read/writes (C++ API)
    EzApp &app = EzApp::instance();
    if (!app.init()) {
        ESP_LOGE("EzApp", "EzApp init failed");
    } else {
        ESP_LOGI("EzApp", "EzApp initialized");
    }

    // start modular tasks
    start_ez_dio_task();  // includes oxygen sensor reading
    // start Y0 toggle test (cycles bits 0..7 on Y0 every 1s)
    start_y0_toggle_test();
    start_modbus_master_task();
    //start_modbus_task(); // UART1 공유 불가: 슬레이브/마스터 동시 실행 금지
    start_analog_task();
// RS485 TXD: GPIO 27
// RS485 RXD: GPIO 14 
// // #define MODBUS_TX_PIN 32
// // #define MODBUS_RX_PIN 33


    // // set D256 to 1234 in EzApp (index-based access)
    // for (size_t i = 0; i < 500; i++) {
    //     app.writeInt16AtIndex(EzApp::D, i, static_cast<int16_t>(i));
    // }
    // for (size_t i = 0; i < 500; i++) {
    //     int16_t val = 0;
    //     app.readInt16AtIndex(EzApp::D, i, val);
    //     ESP_LOGI("EzApp", "D[%d] = %d", i, val);
    // }
    // ESP_LOGI("EzApp", "............");
    ESP_LOGI("EzApp", "EzApp Started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
