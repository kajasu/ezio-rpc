/*!
 * @file DFRobot_EOxygenSensor.cpp
 * @brief Define the basic structure of class DFRobot_EOxygenSensor (ESP-IDF port)
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @version V1.0
 * @date 2021-12-28
 * @url https://github.com/DFRobot/DFRobot_EOxygenSensor
 */
#include "DFRobot_EOxygenSensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "EOxygen";

DFRobot_EOxygenSensor_I2C::DFRobot_EOxygenSensor_I2C(i2c_port_t port, uint8_t addr)
    : _port(port), _addr(addr), _oldVol(0.0f)
{
}

DFRobot_EOxygenSensor_I2C::~DFRobot_EOxygenSensor_I2C()
{
}

esp_err_t DFRobot_EOxygenSensor_I2C::begin(void)
{
    // Test I2C connection by sending device address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "E-Oxygen sensor connected at 0x%02X", _addr);
    } else {
        ESP_LOGW(TAG, "E-Oxygen sensor NOT found at 0x%02X (err=%d)", _addr, err);
    }
    return err;
}

float DFRobot_EOxygenSensor_I2C::readOxygenConcentration(void)
{
    uint8_t buf[3] = {0};
    esp_err_t err = readData(OXYGEN_DATA, buf, 3);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read oxygen data: %d", err);
        return _oldVol;
    }

    float vol = (float)buf[0] + ((float)buf[1]) / 10.0f + ((float)buf[2]) / 100.0f;
    
    // If value is effectively zero, return last valid reading
    if (vol >= -0.00001f && vol <= 0.00001f) {
        return _oldVol;
    } else {
        _oldVol = vol;
        return vol;
    }
}

uint8_t DFRobot_EOxygenSensor_I2C::readCalibrationState(void)
{
    uint8_t buf[1] = {0};
    readData(CALIBRATION_STATE, buf, 1);
    return buf[0];
}

bool DFRobot_EOxygenSensor_I2C::calibration_20_9(void)
{
    uint8_t data = CALIBRATION_20_9;
    writeData(CALIBRATION_SENSOR, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint8_t state = readCalibrationState();
    return (state & CALIBRATION_20_9) != 0;
}

bool DFRobot_EOxygenSensor_I2C::calibration_99_5(void)
{
    uint8_t data = CALIBRATION_99_5;
    writeData(CALIBRATION_SENSOR, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint8_t state = readCalibrationState();
    return (state & CALIBRATION_99_5) != 0;
}

bool DFRobot_EOxygenSensor_I2C::clearCalibration(void)
{
    uint8_t data = CALIBRATION_CLEAR;
    writeData(CALIBRATION_SENSOR, &data, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    uint8_t state = readCalibrationState();
    return (state == 0);
}

esp_err_t DFRobot_EOxygenSensor_I2C::writeData(uint8_t reg, const uint8_t *data, uint8_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (data && len > 0) {
        i2c_master_write(cmd, data, len, true);
    }
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t DFRobot_EOxygenSensor_I2C::readData(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (!data || len == 0) return ESP_ERR_INVALID_ARG;

    // Write register address
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) return err;

    // Read data
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    return err;
}
