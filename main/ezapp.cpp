#include "ezapp.h"
#include "utils.h"
#include <cstring>
#include <cstdlib>
#include <mutex>
#include <algorithm>
#include "esp_log.h"
#include "driver/i2c.h"
#include "i2cdev.h"
#include "ds1307.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <sys/time.h>

namespace {
std::mutex s_rtc_desc_mutex;
i2c_dev_t s_rtc_dev;
bool s_rtc_dev_inited = false;

esp_err_t ensure_rtc_desc_inited()
{
    std::lock_guard<std::mutex> lock(s_rtc_desc_mutex);
    if (s_rtc_dev_inited) return ESP_OK;

    std::memset(&s_rtc_dev, 0, sizeof(s_rtc_dev));
    esp_err_t r = ds1307_init_desc(
        &s_rtc_dev,
        static_cast<i2c_port_t>(EzApp::I2C_PORT),
        static_cast<gpio_num_t>(EzApp::I2C_SDA_IO),
        static_cast<gpio_num_t>(EzApp::I2C_SCL_IO));
    if (r != ESP_OK) return r;

    // Ensure DS1307 access matches our bus config (pull-ups + 100kHz for stability)
    s_rtc_dev.cfg.sda_pullup_en = true;
    s_rtc_dev.cfg.scl_pullup_en = true;
    s_rtc_dev.cfg.master.clk_speed = EzApp::I2C_FREQ_HZ;

    s_rtc_dev_inited = true;
    return ESP_OK;
}
} // namespace

EzApp &EzApp::instance()
{
    static EzApp inst;
    return inst;
}

// Persist/load region descriptor helpers (NVS)
bool EzApp::persistRegionDescriptor(uint8_t regionId, Group g, uint32_t startOffsetBytes, uint32_t countBytes)
{
    // regionId는 uint8_t이므로 항상 0..255 범위 내에 있음
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // try erase + init
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "NVS init failed: %d", err);
        return false;
    }

    nvs_handle_t handle;
    err = nvs_open("ezapp", NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "nvs_open failed: %d", err);
        return false;
    }

    char key_g[32];
    char key_off[32];
    char key_cnt[32];
    snprintf(key_g, sizeof(key_g), "reg%u_g", regionId);
    snprintf(key_off, sizeof(key_off), "reg%u_off", regionId);
    snprintf(key_cnt, sizeof(key_cnt), "reg%u_cnt", regionId);

    err = nvs_set_u32(handle, key_g, static_cast<uint32_t>(g));
    if (err == ESP_OK) err = nvs_set_u32(handle, key_off, startOffsetBytes);
    if (err == ESP_OK) err = nvs_set_u32(handle, key_cnt, countBytes);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "persistRegionDescriptor failed: %d", err);
        return false;
    }
    return true;
}

bool EzApp::loadRegionDescriptor(uint8_t regionId, Group &outGroup, uint32_t &outStartOffsetBytes, uint32_t &outCountBytes)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        (void)nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "NVS init failed: %d", err);
        return false;
    }

    nvs_handle_t handle;
    err = nvs_open("ezapp", NVS_READONLY, &handle);
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "nvs_open failed: %d", err);
        return false;
    }

    char key_g[32];
    char key_off[32];
    char key_cnt[32];
    snprintf(key_g, sizeof(key_g), "reg%u_g", regionId);
    snprintf(key_off, sizeof(key_off), "reg%u_off", regionId);
    snprintf(key_cnt, sizeof(key_cnt), "reg%u_cnt", regionId);

    uint32_t gval = 0;
    uint32_t off = 0;
    uint32_t cnt = 0;
    err = nvs_get_u32(handle, key_g, &gval);
    if (err != ESP_OK) { nvs_close(handle); return false; }
    err = nvs_get_u32(handle, key_off, &off);
    if (err != ESP_OK) { nvs_close(handle); return false; }
    err = nvs_get_u32(handle, key_cnt, &cnt);
    if (err != ESP_OK) { nvs_close(handle); return false; }

    nvs_close(handle);
    if (gval > static_cast<uint32_t>(COUNT - 1)) {
        ESP_LOGW("EzApp", "loadRegionDescriptor: stored group out of range: %u", gval);
        return false;
    }
    outGroup = static_cast<Group>(gval);
    outStartOffsetBytes = off;
    outCountBytes = cnt;
    return true;
}

// Save D130..D159 (30 x int16) into NVS under namespace "EzApp" key "d130_30".
esp_err_t EzApp::saveD130RegionToNVS()
{
    const int count = 30;
    int16_t vals[count];
    for (int i = 0; i < count; ++i) {
        int16_t v = 0;
        // offsets are byte offsets in EzApp API
        (void)readInt16(D, (130 + i) * 2, v);
        vals[i] = v;
    }

    nvs_handle_t handle;
    esp_err_t err = nvs_open("EzApp", NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        nvs_flash_init();
        err = nvs_open("EzApp", NVS_READWRITE, &handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "saveD130RegionToNVS: nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_blob(handle, "d130_30", vals, sizeof(vals));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "saveD130RegionToNVS: nvs_set_blob/commit failed: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("EzApp", "saveD130RegionToNVS: persisted %d registers from D130", count);
    }
    return err;
}

// Load D130..D159 from NVS key "d130_30" and restore into EzApp D-group
esp_err_t EzApp::loadD130RegionFromNVS()
{
    const int count = 30;
    int16_t vals[count];
    nvs_handle_t handle;
    esp_err_t err = nvs_open("EzApp", NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
        nvs_flash_init();
        err = nvs_open("EzApp", NVS_READONLY, &handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "loadD130RegionFromNVS: nvs_open failed: %s", esp_err_to_name(err));
        return err;
    }

    size_t required = 0;
    err = nvs_get_blob(handle, "d130_30", NULL, &required);
    if (err != ESP_OK || required != sizeof(vals)) {
        nvs_close(handle);
        ESP_LOGI("EzApp", "loadD130RegionFromNVS: no saved region or size mismatch (%s)", esp_err_to_name(err));
        return ESP_ERR_NOT_FOUND;
    }

    err = nvs_get_blob(handle, "d130_30", vals, &required);
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGW("EzApp", "loadD130RegionFromNVS: read failed: %s", esp_err_to_name(err));
        return err;
    }

    suppress_persist_ = true;
    for (int i = 0; i < count; ++i) {
        (void)writeInt16(D, (130 + i) * 2, vals[i]);
    }
    suppress_persist_ = false;
    ESP_LOGI("EzApp", "loadD130RegionFromNVS: restored %d registers into D130..D%d", count, 130 + count - 1);
    return ESP_OK;
}

EzApp::EzApp()
    : initialized_(false)
{
    for (int i = 0; i < COUNT; ++i) groups_[i] = nullptr;
    suppress_persist_ = false;
    mutex_ = nullptr;
    // default device statuses
    o2_status_ = DEVICE_KNOWN;
    co2_status_ = DEVICE_KNOWN;
    modbus_status_ = DEVICE_KNOWN;
    o2_err_count_ = 0;
    co2_err_count_ = 0;
    modbus_err_count_ = 0;
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
    
    // Mark initialized before any read/write helpers to avoid recursive init during load.
    initialized_ = true;
    loadD130RegionFromNVS(); // Attempt to restore D130..D159 from NVS on init

    {
        esp_err_t ierr = i2cdev_init();
        if (ierr != ESP_OK) {
            ESP_LOGW("EzApp", "i2cdev_init failed: %d", ierr);
        }
    }

    // Allow power and pull-ups to settle before first I2C transaction
    vTaskDelay(pdMS_TO_TICKS(200));

    // Read RTC first (via i2cdev/ds1307). This ensures i2cdev owns the I2C driver setup
    // before any legacy I2C helper code runs.
    bool rtc_ok = false;
    for (int attempt = 1; attempt <= 5; ++attempt) {
        rtc_ok = updateCurrentTime();
        if (rtc_ok) break;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    if (!rtc_ok) {
        ESP_LOGW("EzApp", "Failed to read RTC time on init");
    }

    // Diagnostic: quick I2C bus scan on configured I2C_PORT to help detect absent devices
    {
        int devices_found = 0;
        for (uint8_t addr = 0x03; addr <= 0x77; ++addr) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, static_cast<uint8_t>((addr << 1) | I2C_MASTER_WRITE), true);
            i2c_master_stop(cmd);
            esp_err_t r = i2c_master_cmd_begin(static_cast<i2c_port_t>(I2C_PORT), cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);
            if (r == ESP_OK) {
                ESP_LOGI("EzApp", "I2C device found at 0x%02X on port %d", addr, I2C_PORT);
                ++devices_found;
            }
        }
        if (devices_found == 0) {
            ESP_LOGW("EzApp", "No I2C devices detected on port %d (SDA=%d,SCL=%d). Check wiring/pull-ups.", I2C_PORT, I2C_SDA_IO, I2C_SCL_IO);
        } else {
            ESP_LOGI("EzApp", "I2C scan: %d device(s) found on port %d", devices_found, I2C_PORT);
        }
    }

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

    // Free DS1307 descriptor last; i2cdev_legacy may uninstall the driver when the
    // last device is removed, which is OK during deinit but must not happen during runtime.
    if (s_rtc_dev_inited) {
        (void)ds1307_free_desc(&s_rtc_dev);
        std::memset(&s_rtc_dev, 0, sizeof(s_rtc_dev));
        s_rtc_dev_inited = false;
    }

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
    // PCF8574: writing 0xFF sets pins high (input mode with pull-ups), 0x00 drives low
    uint8_t in_init = 0xFF;
    uint8_t out_init = 0x00;

    // write to inputs device
    esp_err_t err = i2c_master_write_to_device(static_cast<i2c_port_t>(I2C_PORT), PCF_INPUT_ADDR, &in_init, 1, pdMS_TO_TICKS(1000));
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

bool EzApp::set_rtc_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday)
{
    if (year < 2000) return false;

    if (!initialized_) (void)init();
    esp_err_t r = ensure_rtc_desc_inited();
    if (r != ESP_OK) {
        ESP_LOGE("EzApp", "ensure_rtc_desc_inited failed: %d (%s)", r, esp_err_to_name(r));
        return false;
    }

    struct tm tm = {};
    tm.tm_year = static_cast<int>(year) - 1900;
    tm.tm_mon  = static_cast<int>(month) - 1;
    tm.tm_mday = static_cast<int>(day);
    tm.tm_hour = static_cast<int>(hour);
    tm.tm_min  = static_cast<int>(minute);
    tm.tm_sec  = static_cast<int>(second);
    // weekday is not used by ds1307_set_time API directly; keep cache

    bool ok = false;
    esp_err_t set_err = ds1307_set_time(&s_rtc_dev, &tm);
    if (set_err != ESP_OK) {
        ESP_LOGE("EzApp", "ds1307_set_time failed: %d (%s)", set_err, esp_err_to_name(set_err));
    } else {
        ok = true;
        // Update cached time and system clock after successful write
        rtc_time_.year    = year;
        rtc_time_.month   = month;
        rtc_time_.day     = day;
        rtc_time_.hour    = hour;
        rtc_time_.minute  = minute;
        rtc_time_.second  = second;
        rtc_time_.weekday = weekday;
        rtc_time_.valid   = true;

        time_t t = mktime(&tm);
        if (t != (time_t)-1) {
            struct timeval tv = {};
            tv.tv_sec = t;
            tv.tv_usec = 0;
            (void)settimeofday(&tv, nullptr);
        }
    }
    return ok;
}

// RTC 시간 읽기: DS1307 레지스터 0x00~0x06 (초, 분, 시, 요일, 일, 월, 년)
bool EzApp::updateCurrentTime()
{
    static TickType_t last_update = 0;
    static uint32_t retry_ms = 1000; // success: 1s, failure: 30s
    TickType_t now = xTaskGetTickCount();

    // Until we have at least one valid RTC snapshot, retry more aggressively
    // to avoid staying at 1970 for long due to early-boot I2C timing hiccups.
    const uint32_t effective_retry_ms = rtc_time_.valid ? retry_ms : 200u;
    if ((now - last_update) < pdMS_TO_TICKS(effective_retry_ms)) {
        return rtc_time_.valid;
    }

    uint16_t year = 0;
    uint8_t month = 0, day = 0, hour = 0, minute = 0, second = 0, weekday = 0;
    bool ok = get_rtc_time(year, month, day, hour, minute, second, weekday);
    last_update = now; // update timestamp regardless
    retry_ms = ok ? 1000u : (rtc_time_.valid ? 30000u : 500u);
    if (ok) {
        struct tm tm = {};
        tm.tm_year = static_cast<int>(year) - 1900;
        tm.tm_mon  = static_cast<int>(month) - 1;
        tm.tm_mday = static_cast<int>(day);
        tm.tm_hour = static_cast<int>(hour);
        tm.tm_min  = static_cast<int>(minute);
        tm.tm_sec  = static_cast<int>(second);
        time_t t = mktime(&tm);
        if (t != (time_t)-1) {
            struct timeval tv = {};
            tv.tv_sec = t;
            tv.tv_usec = 0;
            (void)settimeofday(&tv, nullptr);
        }
    }
    return ok;
}

bool EzApp::get_rtc_time(uint16_t &year, uint8_t &month, uint8_t &day,
                         uint8_t &hour, uint8_t &minute, uint8_t &second,
                         uint8_t &weekday)
{
    if (!initialized_) (void)init();
    esp_err_t r = ensure_rtc_desc_inited();
    if (r != ESP_OK) {
        ESP_LOGE("EzApp", "ensure_rtc_desc_inited failed: %d (%s)", r, esp_err_to_name(r));
        return false;
    }

    struct tm tm = {};
    bool ok = false;
    esp_err_t get_err = ds1307_get_time(&s_rtc_dev, &tm);
    if (get_err != ESP_OK) {
        ESP_LOGE("EzApp", "ds1307_get_time failed: %d (%s)", get_err, esp_err_to_name(get_err));
    } else {
        year    = static_cast<uint16_t>(tm.tm_year + 1900);
        month   = static_cast<uint8_t>(tm.tm_mon + 1);
        day     = static_cast<uint8_t>(tm.tm_mday);
        hour    = static_cast<uint8_t>(tm.tm_hour);
        minute  = static_cast<uint8_t>(tm.tm_min);
        second  = static_cast<uint8_t>(tm.tm_sec);
        weekday = static_cast<uint8_t>(tm.tm_wday == 0 ? 7 : tm.tm_wday); // map Sunday(0) -> 7 for device-style

        // 캐시 업데이트
        rtc_time_.year    = year;
        rtc_time_.month   = month;
        rtc_time_.day     = day;
        rtc_time_.hour    = hour;
        rtc_time_.minute  = minute;
        rtc_time_.second  = second;
        rtc_time_.weekday = weekday;
        rtc_time_.valid   = true;
        ok = true;
    }
    return ok;
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
    // Add logging for writeBytes operation
    if (!data || len == 0) return false;
    if (check_bounds_cpp(g, offset, len) != 0) return false;

    if (mutex_) {
        (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    }
    std::memcpy(groups_[g] + offset, data, len);
    if (mutex_) {
        (void)xSemaphoreGive(mutex_);
    }

    // If this write touches D130..D159, persist the region to NVS.
    // Note: offsets for EzApp APIs are BYTE offsets; D130 (word index) => byte offset = 130*2
    if (g == D && initialized_ && !suppress_persist_) {
        const std::size_t region_start = static_cast<std::size_t>(130 * 2);
        const std::size_t region_end = region_start + static_cast<std::size_t>(30 * sizeof(int16_t));
        const std::size_t write_end = offset + len;
        if (offset < region_end && write_end > region_start) {
            ESP_LOGI("EzApp", "writeBytes: persisting D130..D159 region to NVS");
            esp_err_t perr = saveD130RegionToNVS();
            if (perr != ESP_OK) {
                ESP_LOGI("EzApp", "writeBytes: saveD130RegionToNVS failed: %s", esp_err_to_name(perr));
            }
        }
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

bool EzApp::setO2Status(DeviceState s)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    o2_status_ = s;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    // publish to D120
    if (initialized_) {
        (void)writeInt16(D, 120 * 2, static_cast<int16_t>(s));
    }
    return true;
}

EzApp::DeviceState EzApp::getO2Status()
{
    DeviceState s = DEVICE_KNOWN;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    s = o2_status_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return s;
}

bool EzApp::setCO2Status(DeviceState s)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    co2_status_ = s;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    // publish to D122
    if (initialized_) {
        (void)writeInt16(D, 122 * 2, static_cast<int16_t>(s));
    }
    return true;
}

EzApp::DeviceState EzApp::getCO2Status()
{
    DeviceState s = DEVICE_KNOWN;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    s = co2_status_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return s;
}

bool EzApp::setModbusStatus(DeviceState s)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    modbus_status_ = s;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    // publish to D124
    if (initialized_) {
        (void)writeInt16(D, 124 * 2, static_cast<int16_t>(s));
    }
    return true;
}

bool EzApp::incrementO2ErrorCount()
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    ++o2_err_count_;
    uint16_t v = o2_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 121 * 2, static_cast<int16_t>(v));
    return true;
}

bool EzApp::incrementCO2ErrorCount()
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    ++co2_err_count_;
    uint16_t v = co2_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 123 * 2, static_cast<int16_t>(v));
    return true;
}

bool EzApp::incrementModbusErrorCount()
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    ++modbus_err_count_;
    uint16_t v = modbus_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 125 * 2, static_cast<int16_t>(v));
    return true;
}

bool EzApp::setO2ErrorCount(uint16_t v)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    o2_err_count_ = v;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 121 * 2, static_cast<int16_t>(v));
    return true;
}

bool EzApp::setCO2ErrorCount(uint16_t v)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    co2_err_count_ = v;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 123 * 2, static_cast<int16_t>(v));
    return true;
}

bool EzApp::setModbusErrorCount(uint16_t v)
{
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    modbus_err_count_ = v;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    if (initialized_) (void)writeInt16(D, 125 * 2, static_cast<int16_t>(v));
    return true;
}

uint16_t EzApp::getO2ErrorCount()
{
    uint16_t v;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    v = o2_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return v;
}

uint16_t EzApp::getCO2ErrorCount()
{
    uint16_t v;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    v = co2_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return v;
}

uint16_t EzApp::getModbusErrorCount()
{
    uint16_t v;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    v = modbus_err_count_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return v;
}

EzApp::DeviceState EzApp::getModbusStatus()
{
    DeviceState s = DEVICE_KNOWN;
    if (mutex_) (void)xSemaphoreTake(mutex_, portMAX_DELAY);
    s = modbus_status_;
    if (mutex_) (void)xSemaphoreGive(mutex_);
    return s;
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



