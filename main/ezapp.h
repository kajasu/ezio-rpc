// EzApp C++ header (renamed from ezapp.hpp)
#ifndef EZAPP_H
#define EZAPP_H

#include <stdint.h>
#include "esp_err.h"
#ifdef __cplusplus
#include <cstddef>
#include <cstdbool>
// FreeRTOS mutex
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// C++ EzApp class
class EzApp {
public:
    enum Group : int { X = 0, Y = 1, D = 2, COUNT = 3 };

    // Get singleton instance
    static EzApp &instance();

    // Initialize/tear-down (idempotent)
    bool init();
    void deinit();
    
    // Initialize PCF8574 I/O expanders (addresses configured below)
    bool init_pcf8574();


    // Set RTC time. `year` is full year (e.g., 2026). `weekday` is 1..7 (Mon..Sun or device-specific mapping).
    // Returns true on success.
    bool set_rtc_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday = 1);

    // Read RTC time from DS1307. All output parameters are filled on success.
    // `year` is returned as full year (e.g., 2026). `weekday` is 1..7 as stored on chip.
    // Returns true on success. On success, also updates the cached rtc_time_ member.
    bool get_rtc_time(uint16_t &year, uint8_t &month, uint8_t &day, uint8_t &hour, uint8_t &minute, uint8_t &second, uint8_t &weekday);

    // Snapshot of the last successfully read RTC time.
    struct RtcTime {
        uint16_t year    = 0;
        uint8_t  month   = 0;
        uint8_t  day     = 0;
        uint8_t  hour    = 0;
        uint8_t  minute  = 0;
        uint8_t  second  = 0;
        uint8_t  weekday = 0;
        bool     valid   = false; // true once at least one successful read has occurred
    };

    // Returns the last successfully cached RTC time (updated by get_rtc_time on success).
    const RtcTime &rtc_time() const { return rtc_time_; }

    // Read RTC time from hardware and update the cached rtc_time_. Returns true on success.
    bool updateCurrentTime();

    // Device status codes used for sensors and modbus devices
    // 0 = DEVICE_KNOWN, 1 = DEVICE_OK, 2 = DEVICE_ERROR,
    // 3 = DEVICE_INIT_DONE, 4 = DEVICE_INIT_PENDING
    enum DeviceState {
        DEVICE_KNOWN = 0,
        DEVICE_OK = 1,
        DEVICE_ERROR = 2,
        DEVICE_INIT_DONE = 3,
        DEVICE_INIT_PENDING = 4
    };

    // Accessors for O2 / CO2 sensor and Modbus device status
    bool setO2Status(DeviceState s);
    DeviceState getO2Status();
    bool setCO2Status(DeviceState s);
    DeviceState getCO2Status();
    bool setModbusStatus(DeviceState s);
    DeviceState getModbusStatus();

    // Error counter accessors (publish to D121/D123/D125)
    bool incrementO2ErrorCount();
    bool incrementCO2ErrorCount();
    bool incrementModbusErrorCount();
    bool setO2ErrorCount(uint16_t v);
    bool setCO2ErrorCount(uint16_t v);
    bool setModbusErrorCount(uint16_t v);
    uint16_t getO2ErrorCount();
    uint16_t getCO2ErrorCount();
    uint16_t getModbusErrorCount();

    // (log API removed)

    // Standardized D-group offsets for peripherals (bytes)
    static constexpr uint32_t MODBUS_D_BASE = 256; // base for Modbus data
    static constexpr uint32_t MODBUS_SLAVE1_OFFSET = MODBUS_D_BASE * 2; // 30 x int16
    static constexpr uint32_t MODBUS_SLAVE2_OFFSET = MODBUS_D_BASE + (30 * 2);
    // Move Modbus write-source region to 512 per request
    static constexpr uint32_t MODBUS_WRITE_SRC_OFFSET = 512; // 30 x int16 source (60 bytes)

    // Restore oxygen offsets to overlap with Modbus write-source (user requested)
    // NOTE: EzApp::writeInt16/readInt16 offsets are byte offsets.
    static constexpr uint32_t OXYGEN_OFFSET1 = 114 * 2; // uint16 at D114 (byte offset)
    static constexpr uint32_t OXYGEN_OFFSET2 = 115 * 2; // uint16 at D115 (byte offset)


    // Size per group
    static constexpr std::size_t GROUP_SIZE = 2048;

    // PCF8574 device addresses (7-bit). Modify if using different addresses.
    static constexpr uint8_t PCF_INPUT_ADDR = 0x22;
    static constexpr uint8_t PCF_OUTPUT_ADDR = 0x24;

    // I2C defaults (change if your hardware uses other pins/port)
    static constexpr int I2C_PORT = 0; // I2C_NUM_0
    // Updated to user-provided pins
    static constexpr int I2C_SDA_IO = 4;
    static constexpr int I2C_SCL_IO = 15;
    static constexpr int I2C_FREQ_HZ = 100000;
    // If true, suppress any warning about using GPIO15 (strapping pin) for SCL
    static constexpr bool I2C_IGNORE_SCL_STRAPPING_WARNING = true;

private:
    EzApp();
    ~EzApp();
    EzApp(const EzApp &) = delete;
    EzApp &operator=(const EzApp &) = delete;

    uint8_t *groups_[COUNT];
    bool initialized_;
    bool suppress_persist_;
    SemaphoreHandle_t mutex_; // protects read/write access to groups_
    RtcTime rtc_time_;        // cached current RTC time (updated by get_rtc_time)
    
    // Device status fields
    DeviceState o2_status_;
    DeviceState co2_status_;
    DeviceState modbus_status_;
    // Communication error counters
    uint16_t o2_err_count_;
    uint16_t co2_err_count_;
    uint16_t modbus_err_count_;
public:
    // --- Single value read/write (byte offsets) ---
    // NOTE: `offset` is a BYTE offset within the selected group.
    bool readByte(Group g, std::size_t offset, uint8_t &out);
    bool writeByte(Group g, std::size_t offset, uint8_t value);

    bool readInt8(Group g, std::size_t offset, int8_t &out);
    bool writeInt8(Group g, std::size_t offset, int8_t value);

    bool readInt16(Group g, std::size_t offset, int16_t &out);
    bool writeInt16(Group g, std::size_t offset, int16_t value);

    bool readInt32(Group g, std::size_t offset, int32_t &out);
    bool writeInt32(Group g, std::size_t offset, int32_t value);

    bool readFloat(Group g, std::size_t offset, float &out);
    bool writeFloat(Group g, std::size_t offset, float value);

    // --- Index helpers (int16 index) ---
    // `index` is an int16 element index (i.e., index 0 => offset 0, index 1 => offset 2).
    bool readInt16AtIndex(Group g, uint32_t index, int16_t &out);
    bool writeInt16AtIndex(Group g, uint32_t index, int16_t value);

    // --- Group read/write (arrays) ---
    // Arrays use BYTE offset; element size is implied by type.
    bool readBytes(Group g, std::size_t offset, void *out, std::size_t len);
    bool writeBytes(Group g, std::size_t offset, const void *data, std::size_t len);

    bool readByteArray(Group g, std::size_t offset, uint8_t *out, std::size_t count);
    bool writeByteArray(Group g, std::size_t offset, const uint8_t *data, std::size_t count);

    bool readInt8Array(Group g, std::size_t offset, int8_t *out, std::size_t count);
    bool writeInt8Array(Group g, std::size_t offset, const int8_t *data, std::size_t count);

    bool readInt16Array(Group g, std::size_t offset, int16_t *out, std::size_t count);
    bool writeInt16Array(Group g, std::size_t offset, const int16_t *data, std::size_t count);

    bool readInt32Array(Group g, std::size_t offset, int32_t *out, std::size_t count);
    bool writeInt32Array(Group g, std::size_t offset, const int32_t *data, std::size_t count);

    bool readFloatArray(Group g, std::size_t offset, float *out, std::size_t count);
    bool writeFloatArray(Group g, std::size_t offset, const float *data, std::size_t count);

    // --- Bit read/write ---
    // Bit addressing is within the group, bit 0 == LSB of byte 0.
    bool readBit(Group g, uint32_t bitIndex, bool &out);
    bool writeBit(Group g, uint32_t bitIndex, bool value);

    // Packed bit array I/O. Bits are packed LSB-first in each output byte.
    // `outPackedBytes` must be at least (bitCount+7)/8.
    bool readBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, uint8_t *outPackedBytes, std::size_t outPackedLen);
    bool writeBitsPacked(Group g, uint32_t startBit, uint32_t bitCount, const uint8_t *packedBytes, std::size_t packedLen);

    // --- Persistence helpers (store/read group region metadata to flash/NVS) ---
    // Persist a region descriptor identified by `regionId` (0..255). Returns true on success.
    bool persistRegionDescriptor(uint8_t regionId, Group g, uint32_t startOffsetBytes, uint32_t countBytes);

    // Read a persisted region descriptor. Returns true if the entry exists and was read successfully.
    bool loadRegionDescriptor(uint8_t regionId, Group &outGroup, uint32_t &outStartOffsetBytes, uint32_t &outCountBytes);

    // Convenience helpers for Modbus D130..D159 region persistence
    // Save 30 int16 values from D130..D159 into NVS under namespace "modbus" key "d130_30".
    esp_err_t saveD130RegionToNVS();
    // Load 30 int16 values from NVS and restore into D130..D159. Returns ESP_OK if restored.
    esp_err_t loadD130RegionFromNVS();
};

#endif // __cplusplus

#endif // EZAPP_H
