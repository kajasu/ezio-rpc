#include "ezapp.h"
#include <cstring>
#include <cstdlib>
#include <mutex>
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
        groups_[i] = static_cast<int16_t *>(std::malloc(GROUP_SIZE));
        if (!groups_[i]) {
            for (int j = 0; j < i; ++j) std::free(groups_[j]);
            return false;
        }
        std::memset(reinterpret_cast<void *>(groups_[i]), 0, GROUP_SIZE);
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

static inline int check_bounds_cpp(EzApp::Group g, uint32_t offset, uint32_t size)
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

// Internal helpers: read/write raw bytes from group storage
bool EzApp::read_group_bytes(EzApp::Group g, uint32_t offset, void *dest, std::size_t bytes)
{
    if (check_bounds_cpp(g, offset, static_cast<uint32_t>(bytes)) != 0) return false;
    if (!this->mutex_) return false;
    if (xSemaphoreTake(this->mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    std::memcpy(dest, reinterpret_cast<uint8_t *>(this->groups_[g]) + offset, bytes);
    xSemaphoreGive(this->mutex_);
    return true;
}

bool EzApp::write_group_bytes(EzApp::Group g, uint32_t offset, const void *src, std::size_t bytes)
{
    if (check_bounds_cpp(g, offset, static_cast<uint32_t>(bytes)) != 0) return false;
    if (!this->mutex_) return false;
    if (xSemaphoreTake(this->mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) return false;
    std::memcpy(reinterpret_cast<uint8_t *>(this->groups_[g]) + offset, src, bytes);
    xSemaphoreGive(this->mutex_);
    return true;
}

bool EzApp::read_group_int8(Group g, int8_t *dest, std::size_t count)
{
    return read_group_bytes(g, 0, dest, count * sizeof(int8_t));
}

bool EzApp::write_group_int8(Group g, const int8_t *src, std::size_t count)
{
    return write_group_bytes(g, 0, src, count * sizeof(int8_t));
}

bool EzApp::read_group_int16(Group g, int16_t *dest, std::size_t count)
{
    return read_group_bytes(g, 0, dest, count * sizeof(int16_t));
}

bool EzApp::write_group_int16(Group g, const int16_t *src, std::size_t count)
{
    return write_group_bytes(g, 0, src, count * sizeof(int16_t));
}

bool EzApp::read_group_int32(Group g, int32_t *dest, std::size_t count)
{
    return read_group_bytes(g, 0, dest, count * sizeof(int32_t));
}

bool EzApp::write_group_int32(Group g, const int32_t *src, std::size_t count)
{
    return write_group_bytes(g, 0, src, count * sizeof(int32_t));
}

bool EzApp::read_group_float(Group g, float *dest, std::size_t count)
{
    return read_group_bytes(g, 0, dest, count * sizeof(float));
}

bool EzApp::write_group_float(Group g, const float *src, std::size_t count)
{
    return write_group_bytes(g, 0, src, count * sizeof(float));
}

// Legacy-style offset-based single/multi-value helpers (byte offset)
bool EzApp::writeInt8(Group g, uint32_t offset, int8_t v)
{
    return write_group_bytes(g, offset, &v, sizeof(v));
}

bool EzApp::readInt8(Group g, uint32_t offset, int8_t &out)
{
    return read_group_bytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeInt16(Group g, uint32_t offset, int16_t v)
{
    return write_group_bytes(g, offset, &v, sizeof(v));
}

bool EzApp::readInt16(Group g, uint32_t offset, int16_t &out)
{
    return read_group_bytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeInt32(Group g, uint32_t offset, int32_t v)
{
    return write_group_bytes(g, offset, &v, sizeof(v));
}

bool EzApp::readInt32(Group g, uint32_t offset, int32_t &out)
{
    return read_group_bytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeFloat(Group g, uint32_t offset, float v)
{
    return write_group_bytes(g, offset, &v, sizeof(v));
}

bool EzApp::readFloat(Group g, uint32_t offset, float &out)
{
    return read_group_bytes(g, offset, &out, sizeof(out));
}

bool EzApp::writeInt16AtIndex(Group g, std::size_t index, int16_t v)
{
    uint32_t offset = static_cast<uint32_t>(index * sizeof(int16_t));
    return writeInt16(g, offset, v);
}

bool EzApp::readInt16AtIndex(Group g, std::size_t index, int16_t &out)
{
    uint32_t offset = static_cast<uint32_t>(index * sizeof(int16_t));
    return readInt16(g, offset, out);
}

bool EzApp::read_group_value(Group g, std::size_t index, int16_t &out_value)
{
    if (index * sizeof(int16_t) >= GROUP_SIZE) return false;
    return read_group_int16(g, &out_value, 1);
}

bool EzApp::write_group_value(Group g, std::size_t index, int16_t value)
{
    if (index * sizeof(int16_t) >= GROUP_SIZE) return false;
    return write_group_int16(g, &value, 1);
}
