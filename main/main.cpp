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
#include "oxygen.h"
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
    start_ez_dio_task();
    start_modbus_task();
    start_oxygen_task();
    start_analog_task();
    //
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
