#include "dfrobot_oxygen_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <cmath>
#include <string.h>

DfRobotOxygenSensor::DfRobotOxygenSensor() = default;

esp_err_t DfRobotOxygenSensor::probe() const
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (uint8_t)((_addr << 1) | I2C_MASTER_WRITE), true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(_port, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t DfRobotOxygenSensor::begin(i2c_port_t port, uint8_t addr_7bit)
{
    _port = port;
    _addr = addr_7bit;

    esp_err_t err = probe();
    if (err != ESP_OK) return err;

    uint8_t ver = 0;
    err = getVersion(&ver);
    if (err != ESP_OK) return err;

    // Mirror DFRobot intent but be tolerant of variants:
    // - 0xFF => old
    // - anything else => treat as new-ish so probeLife/autocal regs can be attempted.
    if (ver == 0xFF) {
        _versionType = eOldVersion;
    } else {
        _versionType = eNewVersion;
    }

    // Prime key cache (best-effort)
    (void)readFlash();

    return ESP_OK;
}

void DfRobotOxygenSensor::setCurrentMultiplier(float multiplier)
{
    if (multiplier <= 0.0f) return;
    _currentMultiplier = multiplier;
}

void DfRobotOxygenSensor::setFallbackKey(float key)
{
    if (key <= 0.0f) return;
    _fallbackKey = key;
    _fallbackKeyValid = true;
}

void DfRobotOxygenSensor::clearFallbackKey()
{
    _fallbackKeyValid = false;
    _fallbackKey = 0.0f;
}

esp_err_t DfRobotOxygenSensor::writeRegU8(uint8_t reg, uint8_t value) const
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_write_to_device(_port, _addr, payload, sizeof(payload), pdMS_TO_TICKS(500));
}

esp_err_t DfRobotOxygenSensor::writeRegU16LE(uint8_t reg, uint16_t value) const
{
    uint8_t payload[3] = {reg, (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    return i2c_master_write_to_device(_port, _addr, payload, sizeof(payload), pdMS_TO_TICKS(500));
}

esp_err_t DfRobotOxygenSensor::readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t delay_ms_after_ptr_write) const
{
    if (!buf || len == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = i2c_master_write_to_device(_port, _addr, &reg, 1, pdMS_TO_TICKS(500));
    if (err != ESP_OK) return err;
    if (delay_ms_after_ptr_write) vTaskDelay(pdMS_TO_TICKS(delay_ms_after_ptr_write));
    return i2c_master_read_from_device(_port, _addr, buf, len, pdMS_TO_TICKS(500));
}

esp_err_t DfRobotOxygenSensor::readKeyRaw(uint16_t *out_temp_raw, uint8_t out_raw_bytes[2])
{
    uint8_t buf[2] = {0, 0};
    esp_err_t err = readReg(GET_KEY_REGISTER, buf, sizeof(buf), 50);
    if (err != ESP_OK) return err;

    if (out_raw_bytes) {
        out_raw_bytes[0] = buf[0];
        out_raw_bytes[1] = buf[1];
    }

    if (out_temp_raw) {
        *out_temp_raw = (uint16_t)((((uint16_t)buf[1]) << 8) | buf[0]);
    }

    return ESP_OK;
}

esp_err_t DfRobotOxygenSensor::readFlash()
{
    uint16_t temp = 0;
    uint8_t raw[2] = {0, 0};
    esp_err_t err = readKeyRaw(&temp, raw);
    if (err != ESP_OK) return err;

    if (temp == 0) {
        if (_fallbackKeyValid) {
            _key = _fallbackKey;
        } else {
            _key = 20.9f / 120.0f;
        }
    } else {
        _key = (float)temp / 1000.0f;
    }

    return ESP_OK;
}

esp_err_t DfRobotOxygenSensor::calibrate(float vol, float mv)
{
    if (!(vol > 0.0f)) return ESP_ERR_INVALID_ARG;

    // DFRobot does: keyValue = vol*10; if mv == 0 => USER_SET_REGISTER with low byte
    if (std::fabs(mv) < 0.000001f) {
        uint16_t keyValue = (uint16_t)lroundf(vol * 10.0f);
        uint8_t keytemp = (uint8_t)(keyValue & 0xFF);
        return writeRegU8(USER_SET_REGISTER, keytemp);
    }

    // else keyValue = (vol/mv)*1000
    uint16_t keyValue = (uint16_t)lroundf((vol / mv) * 1000.0f);

    if (_versionType == eOldVersion) {
        uint8_t keytemp = (keyValue <= 255) ? (uint8_t)(keyValue & 0xFF) : (uint8_t)255;
        return writeRegU8(AUTUAL_SET_REGISTER, keytemp);
    }

    // new version writes 2 bytes to 0x0C (LE)
    return writeRegU16LE(AUTUAL_SET_REGISTER_, keyValue);
}

esp_err_t DfRobotOxygenSensor::getCurrentData(float *out_current)
{
    if (!out_current) return ESP_ERR_INVALID_ARG;

    uint8_t buf[3] = {0, 0, 0};
    esp_err_t err = readReg(OXYGEN_DATA_REGISTER, buf, sizeof(buf), 100);
    if (err != ESP_OK) return err;

    *out_current = ((float)buf[0]) + ((float)buf[1] / 10.0f) + ((float)buf[2] / 100.0f);
    return ESP_OK;
}

esp_err_t DfRobotOxygenSensor::readOxygenDataRaw(uint8_t out_raw_bytes[3])
{
    if (!out_raw_bytes) return ESP_ERR_INVALID_ARG;
    return readReg(OXYGEN_DATA_REGISTER, out_raw_bytes, 3, 100);
}

esp_err_t DfRobotOxygenSensor::readRegWindow(uint8_t start_reg, uint8_t count, uint8_t *out_bytes)
{
    if (!out_bytes) return ESP_ERR_INVALID_ARG;
    if (count == 0) return ESP_ERR_INVALID_ARG;

    for (uint8_t i = 0; i < count; ++i) {
        uint8_t v = 0;
        esp_err_t err = readReg((uint8_t)(start_reg + i), &v, 1, 10);
        if (err != ESP_OK) return err;
        out_bytes[i] = v;
    }
    return ESP_OK;
}

static float average_float(const float *arr, uint8_t len)
{
    if (!arr || len == 0) return 0.0f;
    double sum = 0.0;
    for (uint8_t i = 0; i < len; ++i) sum += arr[i];
    return (float)(sum / (double)len);
}

esp_err_t DfRobotOxygenSensor::getOxygenData(uint8_t collectNum, float *out_o2_percent)
{
    if (!out_o2_percent) return ESP_ERR_INVALID_ARG;
    if (collectNum == 0) return ESP_ERR_INVALID_ARG;
    if (collectNum > OCOUNT) collectNum = OCOUNT;

    esp_err_t err = readFlash();
    if (err != ESP_OK) return err;

    float current = 0.0f;
    err = getCurrentData(&current);
    if (err != ESP_OK) return err;

    // Mirror DFRobot: oxygen = Key * current. Add optional multiplier for module variants.
    float oxygen = _key * (current * _currentMultiplier);

    // Keep DFRobot-style sliding window (shift right).
    for (int j = (int)collectNum - 1; j > 0; --j) {
        _oxygenData[j] = _oxygenData[j - 1];
    }
    _oxygenData[0] = oxygen;

    if (_filled < collectNum) _filled++;

    *out_o2_percent = average_float(_oxygenData, _filled);
    return ESP_OK;
}

esp_err_t DfRobotOxygenSensor::getVersion(uint8_t *out_version)
{
    if (!out_version) return ESP_ERR_INVALID_ARG;
    uint8_t v = 0;
    esp_err_t err = readReg(VERSION_REGISTER, &v, 1, 0);
    if (err != ESP_OK) return err;
    *out_version = v;
    return ESP_OK;
}

esp_err_t DfRobotOxygenSensor::checkProbeLife(int8_t *out_probe_life)
{
    if (!out_probe_life) return ESP_ERR_INVALID_ARG;

    // Some variants report VERSION_REGISTER=0x00; still try to read probe-life.
    uint8_t v = 0;
    esp_err_t err = readReg(PROBE_LIFE_REGISTER, &v, 1, 0);
    if (err == ESP_OK) {
        *out_probe_life = (int8_t)v;
        return ESP_OK;
    }

    // Fall back to DFRobot semantics when the register isn't supported.
    *out_probe_life = (int8_t)eVersionError;
    return err;
}
