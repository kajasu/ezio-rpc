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
    bool set_rtc_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second, uint8_t weekday = 1);

    // (log API removed)

    // Standardized D-group offsets for peripherals (bytes)
    static constexpr uint32_t MODBUS_D_BASE = 256; // base for Modbus data
    static constexpr uint32_t MODBUS_SLAVE1_OFFSET = MODBUS_D_BASE; // 30 x int16
    static constexpr uint32_t MODBUS_SLAVE2_OFFSET = MODBUS_SLAVE1_OFFSET + (30 * 2);
    // Move Modbus write-source region to 512 per request
    static constexpr uint32_t MODBUS_WRITE_SRC_OFFSET = 512; // 30 x int16 source (60 bytes)

    // Restore oxygen offsets to overlap with Modbus write-source (user requested)
    static constexpr uint32_t OXYGEN_OFFSET1 = 512; // uint16
    static constexpr uint32_t OXYGEN_OFFSET2 = 514; // uint16

    // Typed read/write. Return 0 on success, -1 on error.
    int writeShort(Group g, uint32_t offset, short value);
    int readShort(Group g, uint32_t offset, short &out);

    int writeInt16(Group g, uint32_t offset, int16_t value);
    int readInt16(Group g, uint32_t offset, int16_t &out);

    int writeInt32(Group g, uint32_t offset, int32_t value);
    int readInt32(Group g, uint32_t offset, int32_t &out);

    int writeFloat(Group g, uint32_t offset, float value);
    int readFloat(Group g, uint32_t offset, float &out);

    // Size per group
    static constexpr std::size_t GROUP_SIZE = 4096;

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

    uint8_t *groups_[COUNT];
    bool initialized_;
    SemaphoreHandle_t mutex_; // protects read/write access to groups_
    
};

// C-compatible enum for legacy C wrappers
extern "C" {
typedef enum { EZ_GROUP_X = 0, EZ_GROUP_Y = 1, EZ_GROUP_D = 2, EZ_GROUP_COUNT = 3 } EzGroup;
}

#endif // __cplusplus

#endif // EZAPP_H
