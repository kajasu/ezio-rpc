#include "ezapp.h"
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <algorithm>
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

EzApp &EzApp::instance()
{
    static EzApp inst;
    return inst;
}

EzApp::EzApp()
    : initialized_(false)
{
    for (int i = 0; i < COUNT; ++i) groups_[i] = nullptr;
    mutex_ = nullptr;
}

EzApp::~EzApp()
{
    deinit();
}

bool EzApp::init()
{
    if (initialized_) return true;

    for (int i = 0; i < COUNT; ++i) {
        groups_[i] = static_cast<uint8_t *>(std::calloc(1, GROUP_SIZE));
    }
    initialized_ = true;

    // Initialize PCF8574 expanders; not fatal if it fails, but log it
    if (!init_pcf8574()) {
        ESP_LOGW("EzApp", "PCF8574 init failed or not present");
    } else {
        ESP_LOGI("EzApp", "PCF8574 initialized");
    }

    // create mutex for protecting group reads/writes
    if (!mutex_) {
        mutex_ = xSemaphoreCreateMutex();
        if (!mutex_) {
            ESP_LOGW("EzApp", "Failed to create mutex for EzApp");
        }
    }

    return true;
}

void EzApp::deinit()
{
    if (!initialized_) return;
    for (int i = 0; i < COUNT; ++i) {
        std::free(reinterpret_cast<void *>(groups_[i]));
        groups_[i] = nullptr;
    }
    initialized_ = false;
    
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

static inline int check_bounds_cpp(EzApp::Group g, std::size_t offset, std::size_t size)
{
    if (g < EzApp::X || g >= EzApp::COUNT) return -1;
    if (offset + size > EzApp::GROUP_SIZE) return -1;
    if (!EzApp::instance().init()) return -1;
    return 0;
}

bool EzApp::init_pcf8574()
{
    // Configure I2C master
    i2c_config_t conf;
    std::memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_IO);
    conf.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_IO);
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    esp_err_t err = i2c_param_config(static_cast<i2c_port_t>(I2C_PORT), &conf);
    if (err != ESP_OK) {
        ESP_LOGE("EzApp", "i2c_param_config failed: %d", err);
        return false;
    }
    err = i2c_driver_install(static_cast<i2c_port_t>(I2C_PORT), I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means driver already installed
        ESP_LOGE("EzApp", "i2c_driver_install failed: %d", err);
        return false;
    }

    // PCF8574: writing 0xFF sets pins high (input mode with pull-ups), 0x00 drives low
    uint8_t in_init = 0xFF;
    uint8_t out_init = 0x00;

    // write to inputs device
    err = i2c_master_write_to_device(static_cast<i2c_port_t>(I2C_PORT), PCF_INPUT_ADDR, &in_init, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "write to PCF_INPUT (0x%02x) failed: %d", PCF_INPUT_ADDR, err);
    }

    // write to outputs device
    err = i2c_master_write_to_device(static_cast<i2c_port_t>(I2C_PORT), PCF_OUTPUT_ADDR, &out_init, 1, pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "write to PCF_OUTPUT (0x%02x) failed: %d", PCF_OUTPUT_ADDR, err);
    }

    
    return true;
}

static inline uint8_t to_bcd(uint8_t v)
{
    return static_cast<uint8_t>(((v / 10) << 4) | (v % 10));
}

bool EzApp::init_rtc()
{
    // Ensure I2C driver installed/configured (safe to call multiple times)
    i2c_config_t conf;
    std::memset(&conf, 0, sizeof(conf));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = static_cast<gpio_num_t>(I2C_SDA_IO);
    conf.scl_io_num = static_cast<gpio_num_t>(I2C_SCL_IO);
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ_HZ;
    esp_err_t err = i2c_param_config(static_cast<i2c_port_t>(I2C_PORT), &conf);
    if (err != ESP_OK) {
        ESP_LOGE("EzApp", "i2c_param_config failed for RTC: %d", err);
        return false;
    }
    err = i2c_driver_install(static_cast<i2c_port_t>(I2C_PORT), I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("EzApp", "i2c_driver_install failed for RTC: %d", err);
        return false;
    }
    return true;
}

bool EzApp::set_rtc_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday)
{
    // DS1307 I2C address
    const uint8_t DS1307_ADDR = 0x68;
    // Validate and normalize year -> two-digit year
    if (year < 2000) return false;
    uint8_t yy = static_cast<uint8_t>(year % 100);

    uint8_t buf[8];
    buf[0] = 0x00; // start at register 0
    // seconds: clear CH (bit7) to enable oscillator
    buf[1] = to_bcd(static_cast<uint8_t>(second)) & 0x7F;
    buf[2] = to_bcd(static_cast<uint8_t>(minute));
    // hours: keep 24-hour mode (bit6 = 0)
    buf[3] = to_bcd(static_cast<uint8_t>(hour)) & 0x3F;
    // day of week (1-7)
    buf[4] = to_bcd(weekday);
    // date (day of month)
    buf[5] = to_bcd(static_cast<uint8_t>(day));
    // month
    buf[6] = to_bcd(static_cast<uint8_t>(month));
    // year (00-99)
    buf[7] = to_bcd(yy);

    esp_err_t err = i2c_master_write_to_device(static_cast<i2c_port_t>(I2C_PORT), DS1307_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(1000));
    if (err != ESP_OK) {
        ESP_LOGE("EzApp", "Failed to write RTC time: %d", err);
        return false;
    }
    return true;
}

bool EzApp::readBytes(Group g, std::size_t offset, void *out, std::size_t len)
{
    if (!out || len == 0) return false;
    if (check_bounds_cpp(g, offset, len) != 0) return false;

    if (mutex_) {
        (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    std::memcpy(out, groups_[g] + offset, len);
    if (mutex_) {
        (void)xSemaphoreGive(mutex_);
    }
    return true;
}

bool EzApp::writeBytes(Group g, std::size_t offset, const void *data, std::size_t len)
{
    if (!data || len == 0) return false;
    if (check_bounds_cpp(g, offset, len) != 0) return false;

    if (mutex_) {
        (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    std::memcpy(groups_[g] + offset, data, len);
    if (mutex_) {
        (void)xSemaphoreGive(mutex_);
    }
    return true;
}

bool EzApp::readByte(Group g, std::size_t offset, uint8_t &out)
{
    return readBytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeByte(Group g, std::size_t offset, uint8_t value)
{
    return writeBytes(g, offset, &value, sizeof(value));
}

bool EzApp::readInt8(Group g, std::size_t offset, int8_t &out)
{
    return readBytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeInt8(Group g, std::size_t offset, int8_t value)
{
    return writeBytes(g, offset, &value, sizeof(value));
}

bool EzApp::readInt16(Group g, std::size_t offset, int16_t &out)
{
    uint8_t tmp[sizeof(int16_t)];
    if (!readBytes(g, offset, tmp, sizeof(tmp))) return false;
    std::memcpy(&out, tmp, sizeof(out));
    return true;
}

bool EzApp::writeInt16(Group g, std::size_t offset, int16_t value)
{
    uint8_t tmp[sizeof(int16_t)];
    std::memcpy(tmp, &value, sizeof(value));
    return writeBytes(g, offset, tmp, sizeof(tmp));
}

bool EzApp::readInt32(Group g, std::size_t offset, int32_t &out)
{
    uint8_t tmp[sizeof(int32_t)];
    if (!readBytes(g, offset, tmp, sizeof(tmp))) return false;
    std::memcpy(&out, tmp, sizeof(out));
    return true;
}

bool EzApp::writeInt32(Group g, std::size_t offset, int32_t value)
{
    uint8_t tmp[sizeof(int32_t)];
    std::memcpy(tmp, &value, sizeof(value));
    return writeBytes(g, offset, tmp, sizeof(tmp));
}

bool EzApp::readFloat(Group g, std::size_t offset, float &out)
{
    uint8_t tmp[sizeof(float)];
    if (!readBytes(g, offset, tmp, sizeof(tmp))) return false;
    std::memcpy(&out, tmp, sizeof(out));
    return true;
}

bool EzApp::writeFloat(Group g, std::size_t offset, float value)
{
    uint8_t tmp[sizeof(float)];
    std::memcpy(tmp, &value, sizeof(value));
    return writeBytes(g, offset, tmp, sizeof(tmp));
}

bool EzApp::readInt16AtIndex(Group g, uint32_t index, int16_t &out)
{
    return readInt16(g, static_cast<std::size_t>(index) * sizeof(int16_t), out);
}

bool EzApp::writeInt16AtIndex(Group g, uint32_t index, int16_t value)
{
    return writeInt16(g, static_cast<std::size_t>(index) * sizeof(int16_t), value);
}

bool EzApp::readByteArray(Group g, std::size_t offset, uint8_t *out, std::size_t count)
{
    return readBytes(g, offset, out, count);
}

bool EzApp::writeByteArray(Group g, std::size_t offset, const uint8_t *data, std::size_t count)
{
    return writeBytes(g, offset, data, count);
}

bool EzApp::readInt8Array(Group g, std::size_t offset, int8_t *out, std::size_t count)
{
    return readBytes(g, offset, out, count * sizeof(int8_t));
}

bool EzApp::writeInt8Array(Group g, std::size_t offset, const int8_t *data, std::size_t count)
{
    return writeBytes(g, offset, data, count * sizeof(int8_t));
}

bool EzApp::readInt16Array(Group g, std::size_t offset, int16_t *out, std::size_t count)
{
    return readBytes(g, offset, out, count * sizeof(int16_t));
}

bool EzApp::writeInt16Array(Group g, std::size_t offset, const int16_t *data, std::size_t count)
{
    return writeBytes(g, offset, data, count * sizeof(int16_t));
}

bool EzApp::readInt32Array(Group g, std::size_t offset, int32_t *out, std::size_t count)
{
    return readBytes(g, offset, out, count * sizeof(int32_t));
}

bool EzApp::writeInt32Array(Group g, std::size_t offset, const int32_t *data, std::size_t count)
{
    return writeBytes(g, offset, data, count * sizeof(int32_t));
}

bool EzApp::readFloatArray(Group g, std::size_t offset, float *out, std::size_t count)
{
    return readBytes(g, offset, out, count * sizeof(float));
}

bool EzApp::writeFloatArray(Group g, std::size_t offset, const float *data, std::size_t count)
{
    return writeBytes(g, offset, data, count * sizeof(float));
}

bool EzApp::readBit(Group g, uint32_t bitIndex, bool &out)
{
    std::size_t byte_off = static_cast<std::size_t>(bitIndex / 8u);
    uint8_t mask = static_cast<uint8_t>(1u << (bitIndex % 8u));
    uint8_t b = 0;
    if (!readByte(g, byte_off, b)) return false;
    out = (b & mask) != 0;
    return true;
}

bool EzApp::writeBit(Group g, uint32_t bitIndex, bool value)
{
    std::size_t byte_off = static_cast<std::size_t>(bitIndex / 8u);
    uint8_t mask = static_cast<uint8_t>(1u << (bitIndex % 8u));
    uint8_t b = 0;
    if (!readByte(g, byte_off, b)) return false;
    if (value) b |= mask;
    else b &= static_cast<uint8_t>(~mask);
    return writeByte(g, byte_off, b);
}

bool EzApp::readBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, uint8_t *outPackedBytes, std::size_t outPackedLen)
{
    if (!outPackedBytes) return false;
    std::size_t need = (static_cast<std::size_t>(bitCount) + 7u) / 8u;
    if (outPackedLen < need) return false;
    std::memset(outPackedBytes, 0, outPackedLen);

    for (uint32_t i = 0; i < bitCount; ++i) {
        bool bit = false;
        if (!readBit(g, startBit + i, bit)) return false;
        if (bit) {
            outPackedBytes[i / 8u] |= static_cast<uint8_t>(1u << (i % 8u));
        }
    }
    return true;
}

bool EzApp::writeBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, const uint8_t *packedBytes, std::size_t packedLen)
{
    if (!packedBytes) return false;
    std::size_t need = (static_cast<std::size_t>(bitCount) + 7u) / 8u;
    if (packedLen < need) return false;

    for (uint32_t i = 0; i < bitCount; ++i) {
        bool bit = (packedBytes[i / 8u] & static_cast<uint8_t>(1u << (i % 8u))) != 0;
        if (!writeBit(g, startBit + i, bit)) return false;
    }
    return true;
}



