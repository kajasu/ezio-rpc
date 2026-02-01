// EzApp C++ header (renamed from ezapp.hpp)
#ifndef EZAPP_H
#define EZAPP_H

#include <stdint.h>
#ifdef __cplusplus
#include <cstddef>
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

    // RTC (DS1307) support
    // Initialize RTC over I2C (ensures driver is configured). Returns true on success.
    bool init_rtc();

    // Set RTC time. `year` is full year (e.g., 2026). `weekday` is 1..7 (Mon..Sun or device-specific mapping).
    // Returns true on success.
    bool set_rtc_time(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday = 1);

    // (log API removed)

    // Standardized D-group offsets for peripherals (bytes)
    static constexpr uint32_t MODBUS_D_BASE = 256; // base for Modbus data
    static constexpr uint32_t MODBUS_SLAVE1_OFFSET = MODBUS_D_BASE * 2; // 30 x int16
    static constexpr uint32_t MODBUS_SLAVE2_OFFSET = MODBUS_D_BASE + (30 * 2);
    // Move Modbus write-source region to 512 per request
    static constexpr uint32_t MODBUS_WRITE_SRC_OFFSET = 512; // 30 x int16 source (60 bytes)

    // Restore oxygen offsets to overlap with Modbus write-source (user requested)
    static constexpr uint32_t OXYGEN_OFFSET1 = 512; // uint16
    static constexpr uint32_t OXYGEN_OFFSET2 = 514; // uint16


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
    static constexpr int I2C_FREQ_HZ = 400000;
    // If true, suppress any warning about using GPIO15 (strapping pin) for SCL
    static constexpr bool I2C_IGNORE_SCL_STRAPPING_WARNING = true;

private:
    EzApp();
    ~EzApp();
    EzApp(const EzApp &) = delete;
    EzApp &operator=(const EzApp &) = delete;

    int16_t *groups_[COUNT];
    bool initialized_;
    SemaphoreHandle_t mutex_; // protects read/write access to groups_
    // Internal helpers for byte-level group access (private). `offset` is byte offset within group.
    bool read_group_bytes(Group g, uint32_t offset, void *dest, std::size_t bytes);
    bool write_group_bytes(Group g, uint32_t offset, const void *src, std::size_t bytes);
    
public:
    // Read/write typed elements from/to a group's raw storage. `count` is number of elements.
    // All methods return false on failure (invalid group, out-of-bounds, not initialized).
    bool read_group_int8(Group g, int8_t *dest, std::size_t count);
    bool write_group_int8(Group g, const int8_t *src, std::size_t count);

    bool read_group_int16(Group g, int16_t *dest, std::size_t count);
    bool write_group_int16(Group g, const int16_t *src, std::size_t count);

    bool read_group_int32(Group g, int32_t *dest, std::size_t count);
    bool write_group_int32(Group g, const int32_t *src, std::size_t count);

    bool read_group_float(Group g, float *dest, std::size_t count);
    bool write_group_float(Group g, const float *src, std::size_t count);

    // Legacy offset-based access (offset in bytes from start of group)
    bool writeInt8(Group g, uint32_t offset, int8_t v);
    bool readInt8(Group g, uint32_t offset, int8_t &out);
    bool writeInt16(Group g, uint32_t offset, int16_t v);
    bool readInt16(Group g, uint32_t offset, int16_t &out);
    bool writeInt32(Group g, uint32_t offset, int32_t v);
    bool readInt32(Group g, uint32_t offset, int32_t &out);
    bool writeFloat(Group g, uint32_t offset, float v);
    bool readFloat(Group g, uint32_t offset, float &out);

    // Element-index based helpers (index is element index, not byte offset)
    bool writeInt16AtIndex(Group g, std::size_t index, int16_t v);
    bool readInt16AtIndex(Group g, std::size_t index, int16_t &out);

    // Convenience single-element accessors. Return false if out of range or not initialized.
    bool read_group_value(Group g, std::size_t index, int16_t &out_value);
    bool write_group_value(Group g, std::size_t index, int16_t value);
    
};

#endif // __cplusplus

#endif // EZAPP_H
