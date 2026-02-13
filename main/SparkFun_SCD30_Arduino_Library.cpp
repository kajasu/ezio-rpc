/*
  SCD30 CO2 Sensor Library (ESP-IDF Port)
  Based on SparkFun SCD30 Arduino Library
*/

#include "SparkFun_SCD30_Arduino_Library.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SCD30";

SCD30::SCD30(void)
{
    // Constructor
}

esp_err_t SCD30::begin(i2c_port_t wirePort, bool autoCalibrate, bool measBegin)
{
    _i2cPort = wirePort;

    if (isConnected() == false)
        return ESP_FAIL;

    if (measBegin == false)
        return ESP_OK;

    // Check for device to respond correctly
    if (beginMeasuring() == true)
    {
        setMeasurementInterval(2);             // 2 seconds between measurements
        setAutoSelfCalibration(autoCalibrate); // Enable auto-self-calibration
        return ESP_OK;
    }

    return ESP_FAIL;
}

bool SCD30::isConnected()
{
    uint16_t fwVer;
    if (getFirmwareVersion(&fwVer) == false)
        return false;

    ESP_LOGI(TAG, "Firmware version: 0x%04X", fwVer);
    return true;
}

uint16_t SCD30::getCO2(void)
{
    if (co2HasBeenReported == true)
    {
        if (readMeasurement() == false)
            if (!_useStaleData)
                co2 = 0;
    }

    co2HasBeenReported = true;
    return (uint16_t)co2;
}

float SCD30::getHumidity(void)
{
    if (humidityHasBeenReported == true)
        if (readMeasurement() == false)
            if (!_useStaleData)
                humidity = 0;

    humidityHasBeenReported = true;
    return humidity;
}

float SCD30::getTemperature(void)
{
    if (temperatureHasBeenReported == true)
        if (readMeasurement() == false)
            if (!_useStaleData)
                temperature = 0;

    temperatureHasBeenReported = true;
    return temperature;
}

bool SCD30::setAutoSelfCalibration(bool enable)
{
    if (enable)
        return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 1);
    else
        return sendCommand(COMMAND_AUTOMATIC_SELF_CALIBRATION, 0);
}

bool SCD30::setForcedRecalibrationFactor(uint16_t concentration)
{
    if (concentration < 400 || concentration > 2000)
        return false;
    return sendCommand(COMMAND_SET_FORCED_RECALIBRATION_FACTOR, concentration);
}

float SCD30::getTemperatureOffset(void)
{
    uint16_t response = readRegister(COMMAND_SET_TEMPERATURE_OFFSET);

    union
    {
        int16_t signed16;
        uint16_t unsigned16;
    } signedUnsigned;
    signedUnsigned.unsigned16 = response;

    return (((float)signedUnsigned.signed16) / 100.0);
}

bool SCD30::setTemperatureOffset(float tempOffset)
{
    if (tempOffset < 0.0)
        return false;

    uint16_t value = tempOffset * 100;
    return sendCommand(COMMAND_SET_TEMPERATURE_OFFSET, value);
}

uint16_t SCD30::getAltitudeCompensation(void)
{
    return readRegister(COMMAND_SET_ALTITUDE_COMPENSATION);
}

bool SCD30::setAltitudeCompensation(uint16_t altitude)
{
    return sendCommand(COMMAND_SET_ALTITUDE_COMPENSATION, altitude);
}

bool SCD30::setAmbientPressure(uint16_t pressure_mbar)
{
    if (pressure_mbar < 700 || pressure_mbar > 1200)
        return false;
    return sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressure_mbar);
}

void SCD30::reset()
{
    sendCommand(COMMAND_RESET);
}

bool SCD30::getAutoSelfCalibration()
{
    uint16_t response = readRegister(COMMAND_AUTOMATIC_SELF_CALIBRATION);
    return (response == 1);
}

bool SCD30::beginMeasuring(uint16_t pressureOffset)
{
    return (sendCommand(COMMAND_CONTINUOUS_MEASUREMENT, pressureOffset));
}

bool SCD30::beginMeasuring(void)
{
    return (beginMeasuring(0));
}

bool SCD30::StopMeasurement(void)
{
    return (sendCommand(COMMAND_STOP_MEAS));
}

bool SCD30::setMeasurementInterval(uint16_t interval)
{
    return sendCommand(COMMAND_SET_MEASUREMENT_INTERVAL, interval);
}

uint16_t SCD30::getMeasurementInterval(void)
{
    uint16_t interval = 0;
    getSettingValue(COMMAND_SET_MEASUREMENT_INTERVAL, &interval);
    return (interval);
}

bool SCD30::dataAvailable()
{
    uint16_t response = readRegister(COMMAND_GET_DATA_READY);
    return (response == 1);
}

bool SCD30::readMeasurement()
{
    if (dataAvailable() == false)
        return false;

    ByteToFl tempCO2 = {0};
    ByteToFl tempHumidity = {0};
    ByteToFl tempTemperature = {0};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, COMMAND_READ_MEASUREMENT >> 8, true);
    i2c_master_write_byte(cmd, COMMAND_READ_MEASUREMENT & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send read command: %d", err);
        return false;
    }

    // SCD30 needs time to prepare data (uses clock stretching)
    vTaskDelay(pdMS_TO_TICKS(30));

    uint8_t data[18];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_READ, true);
    // Read all 18 bytes: bytes 0-16 with ACK, byte 17 with NACK
    for (int i = 0; i < 17; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[17], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read measurement data: %d", err);
        return false;
    }

    bool error = false;
    uint8_t bytesToCrc[2];
    for (uint8_t x = 0; x < 18; x++)
    {
        uint8_t incoming = data[x];

        switch (x)
        {
        case 0:
        case 1:
        case 3:
        case 4:
            tempCO2.array[x < 3 ? 3 - x : 4 - x] = incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        case 6:
        case 7:
        case 9:
        case 10:
            tempTemperature.array[x < 9 ? 9 - x : 10 - x] = incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        case 12:
        case 13:
        case 15:
        case 16:
            tempHumidity.array[x < 15 ? 15 - x : 16 - x] = incoming;
            bytesToCrc[x % 3] = incoming;
            break;
        default:
            // Validate CRC
            uint8_t foundCrc = computeCRC8(bytesToCrc, 2);
            if (foundCrc != incoming)
            {
                ESP_LOGW(TAG, "CRC error at byte %d: expected 0x%02X, got 0x%02X", x, foundCrc, incoming);
                error = true;
            }
            break;
        }
    }

    if (error)
        return false;

    co2 = tempCO2.value;
    temperature = tempTemperature.value;
    humidity = tempHumidity.value;

    co2HasBeenReported = false;
    humidityHasBeenReported = false;
    temperatureHasBeenReported = false;

    return true;
}

bool SCD30::getSettingValue(uint16_t registerAddress, uint16_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress >> 8, true);
    i2c_master_write_byte(cmd, registerAddress & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
        return false;

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t data[3];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[2], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
        return false;

    *val = (uint16_t)data[0] << 8 | data[1];
    uint8_t expectedCRC = computeCRC8(data, 2);
    return (data[2] == expectedCRC);
}

uint16_t SCD30::readRegister(uint16_t registerAddress)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, registerAddress >> 8, true);
    i2c_master_write_byte(cmd, registerAddress & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
        return 0;

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t data[2];
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, &data[0], I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &data[1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK)
        return 0;

    return ((uint16_t)data[0] << 8 | data[1]);
}

bool SCD30::sendCommand(uint16_t command, uint16_t arguments)
{
    uint8_t data[2];
    data[0] = arguments >> 8;
    data[1] = arguments & 0xFF;
    uint8_t crc = computeCRC8(data, 2);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command >> 8, true);
    i2c_master_write_byte(cmd, command & 0xFF, true);
    i2c_master_write_byte(cmd, arguments >> 8, true);
    i2c_master_write_byte(cmd, arguments & 0xFF, true);
    i2c_master_write_byte(cmd, crc, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    return (err == ESP_OK);
}

bool SCD30::sendCommand(uint16_t command)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SCD30_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command >> 8, true);
    i2c_master_write_byte(cmd, command & 0xFF, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_i2cPort, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    return (err == ESP_OK);
}

uint8_t SCD30::computeCRC8(uint8_t data[], uint8_t len)
{
    uint8_t crc = 0xFF;

    for (uint8_t x = 0; x < len; x++)
    {
        crc ^= data[x];

        for (uint8_t i = 0; i < 8; i++)
        {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }

    return crc;
}
