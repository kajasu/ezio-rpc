#pragma once

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

class DfRobotOxygenSensor {
public:
    static constexpr uint8_t ADDRESS_0 = 0x70;
    static constexpr uint8_t ADDRESS_1 = 0x71;
    static constexpr uint8_t ADDRESS_2 = 0x72;
    static constexpr uint8_t ADDRESS_3 = 0x73;

    static constexpr uint8_t OXYGEN_DATA_REGISTER = 0x03;
    static constexpr uint8_t USER_SET_REGISTER = 0x08;
    static constexpr uint8_t AUTUAL_SET_REGISTER = 0x09;
    static constexpr uint8_t GET_KEY_REGISTER = 0x0A;
    static constexpr uint8_t AUTUAL_SET_REGISTER_ = 0x0C;
    static constexpr uint8_t PROBE_LIFE_REGISTER = 0x0E;
    static constexpr uint8_t VERSION_REGISTER = 0x0F;

    enum Version : uint8_t {
        eOldVersion = 0x00,
        eNewVersion = 0x01,
    };

    // Matches DFRobot semantics:
    // - exhausted: 0
    // - normal: 1
    // - version error: -1 (old version has no probe-life register)
    enum ProbeLife : int8_t {
        eVersionError = -1,
        eProbeExhausted = 0,
        eProbeNormal = 1,
    };

    DfRobotOxygenSensor();

    esp_err_t begin(i2c_port_t port, uint8_t addr_7bit);

    // Same behavior as DFRobot: if mv is ~0, writes USER_SET_REGISTER with vol*10.
    // Otherwise uses AUTUAL_SET_REGISTER (old) or AUTUAL_SET_REGISTER_ (new).
    esp_err_t calibrate(float vol, float mv = 0.0f);

    // Equivalent to DFRobot getOxygenData(collectNum).
    // Returns smoothed oxygen percentage.
    esp_err_t getOxygenData(uint8_t collectNum, float *out_o2_percent);

    // Equivalent to DFRobot getCurrentData(). Returns raw current value (b0 + b1/10 + b2/100).
    esp_err_t getCurrentData(float *out_current);

    // Reads GET_KEY_REGISTER and updates internal key.
    esp_err_t readFlash();

    esp_err_t getVersion(uint8_t *out_version);
    esp_err_t checkProbeLife(int8_t *out_probe_life);

    // Debug helpers
    esp_err_t readOxygenDataRaw(uint8_t out_raw_bytes[3]);
    esp_err_t readRegWindow(uint8_t start_reg, uint8_t count, uint8_t *out_bytes);

    // For modules that never expose/store key (always 0x0000), you can provide a software key.
    void setFallbackKey(float key);
    void clearFallbackKey();

    void setCurrentMultiplier(float multiplier);

    float key() const { return _key; }
    uint8_t versionType() const { return _versionType; }

    // Convenience: read raw key bytes/temp (does not change fallback setting).
    esp_err_t readKeyRaw(uint16_t *out_temp_raw, uint8_t out_raw_bytes[2]);

private:
    static constexpr int OCOUNT = 100;

    esp_err_t probe() const;
    esp_err_t writeRegU8(uint8_t reg, uint8_t value) const;
    esp_err_t writeRegU16LE(uint8_t reg, uint16_t value) const;
    esp_err_t readReg(uint8_t reg, uint8_t *buf, size_t len, uint32_t delay_ms_after_ptr_write) const;

    i2c_port_t _port = I2C_NUM_0;
    uint8_t _addr = ADDRESS_0;

    uint8_t _versionType = eOldVersion;

    float _key = 0.0f;
    float _currentMultiplier = 1.0f;

    bool _fallbackKeyValid = false;
    float _fallbackKey = 0.0f;

    float _oxygenData[OCOUNT] = {0.0f};
    uint8_t _filled = 0;
};
